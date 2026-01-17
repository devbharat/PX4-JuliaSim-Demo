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
export LogColumn, LogSchema, csv_schema, csv_header_line, write_csv_header

# Update this when CSV columns are added, removed, or renamed.
const CSV_SCHEMA_VERSION = 2

############################
# Explicit log schema
############################

"""One column in a structured log schema."""
struct LogColumn
    name::String
    unit::String
    desc::String
end

"""A versioned log schema.

This is intentionally lightweight so it can be used by:
- the in-memory `SimLog`
- the streaming `CSVLogSink`
- a future HDF5 sink
"""
struct LogSchema
    name::String
    version::Int
    columns::Vector{LogColumn}
end

"""Canonical CSV schema used by `CSVLogSink` and `write_csv`.

Notes
-----
- This schema is defined *once* and reused to generate headers.
- The CSV header line is the ordered list of `columns[i].name`.
"""
const CSV_LOG_SCHEMA = LogSchema(
    "px4lockstep_sim_csv",
    CSV_SCHEMA_VERSION,
    LogColumn[
        # time
        LogColumn("time_s", "s", "simulation time"),
        # truth (NED)
        LogColumn("pos_x", "m", "truth position NED x"),
        LogColumn("pos_y", "m", "truth position NED y"),
        LogColumn("pos_z", "m", "truth position NED z"),
        LogColumn("vel_x", "m/s", "truth velocity NED x"),
        LogColumn("vel_y", "m/s", "truth velocity NED y"),
        LogColumn("vel_z", "m/s", "truth velocity NED z"),
        # attitude (q_bn)
        LogColumn("q_w", "-", "truth attitude quaternion (w)"),
        LogColumn("q_x", "-", "truth attitude quaternion (x)"),
        LogColumn("q_y", "-", "truth attitude quaternion (y)"),
        LogColumn("q_z", "-", "truth attitude quaternion (z)"),
        # body rates
        LogColumn("p", "rad/s", "truth body rate p"),
        LogColumn("q", "rad/s", "truth body rate q"),
        LogColumn("r", "rad/s", "truth body rate r"),
        # setpoints (from PX4)
        LogColumn("pos_sp_x", "m", "position setpoint NED x"),
        LogColumn("pos_sp_y", "m", "position setpoint NED y"),
        LogColumn("pos_sp_z", "m", "position setpoint NED z"),
        LogColumn("vel_sp_x", "m/s", "velocity setpoint NED x"),
        LogColumn("vel_sp_y", "m/s", "velocity setpoint NED y"),
        LogColumn("vel_sp_z", "m/s", "velocity setpoint NED z"),
        LogColumn("acc_sp_x", "m/s^2", "acceleration setpoint NED x"),
        LogColumn("acc_sp_y", "m/s^2", "acceleration setpoint NED y"),
        LogColumn("acc_sp_z", "m/s^2", "acceleration setpoint NED z"),
        LogColumn("yaw_sp", "rad", "yaw setpoint"),
        LogColumn("yawspeed_sp", "rad/s", "yawspeed setpoint"),
        # motor command (PX4 actuator_motors 1..12)
        LogColumn("m1", "norm", "motor command 1"),
        LogColumn("m2", "norm", "motor command 2"),
        LogColumn("m3", "norm", "motor command 3"),
        LogColumn("m4", "norm", "motor command 4"),
        LogColumn("m5", "norm", "motor command 5"),
        LogColumn("m6", "norm", "motor command 6"),
        LogColumn("m7", "norm", "motor command 7"),
        LogColumn("m8", "norm", "motor command 8"),
        LogColumn("m9", "norm", "motor command 9"),
        LogColumn("m10", "norm", "motor command 10"),
        LogColumn("m11", "norm", "motor command 11"),
        LogColumn("m12", "norm", "motor command 12"),
        # rotor outputs (optional convenience)
        LogColumn("rotor_w1", "rad/s", "rotor omega 1"),
        LogColumn("rotor_w2", "rad/s", "rotor omega 2"),
        LogColumn("rotor_w3", "rad/s", "rotor omega 3"),
        LogColumn("rotor_w4", "rad/s", "rotor omega 4"),
        LogColumn("rotor_T1", "N", "rotor thrust 1"),
        LogColumn("rotor_T2", "N", "rotor thrust 2"),
        LogColumn("rotor_T3", "N", "rotor thrust 3"),
        LogColumn("rotor_T4", "N", "rotor thrust 4"),
        # environment
        LogColumn("wind_x", "m/s", "wind NED x"),
        LogColumn("wind_y", "m/s", "wind NED y"),
        LogColumn("wind_z", "m/s", "wind NED z"),
        LogColumn("rho", "kg/m^3", "air density"),
        LogColumn("air_bx", "m/s", "air-relative velocity body x"),
        LogColumn("air_by", "m/s", "air-relative velocity body y"),
        LogColumn("air_bz", "m/s", "air-relative velocity body z"),
        # battery
        LogColumn("batt_v", "V", "battery voltage"),
        LogColumn("batt_a", "A", "battery current"),
        LogColumn("batt_rem", "frac", "battery remaining fraction"),
        LogColumn("batt_warn", "enum", "battery warning"),
        # px4 status
        LogColumn("nav_state", "enum", "PX4 nav_state"),
        LogColumn("arming_state", "enum", "PX4 arming_state"),
        LogColumn("mission_seq", "idx", "mission sequence index"),
        LogColumn("mission_count", "count", "mission item count"),
        LogColumn("mission_finished", "bool", "mission finished flag"),
        # lockstep clock
        LogColumn("time_us", "us", "authoritative lockstep clock"),
    ],
)

"""Return the canonical CSV log schema."""
csv_schema() = CSV_LOG_SCHEMA

"""Return the CSV header line (comma-separated column names)."""
csv_header_line(schema::LogSchema = CSV_LOG_SCHEMA) =
    join((c.name for c in schema.columns), ",")

"""Write CSV header comments + column header line.

This keeps the on-disk format self-describing while remaining dependency-light.

Header format
-------------
- `# schema_version=<int>`
- `# schema_name=<string>`
- `<csv header line>`
"""
function write_csv_header(io::IO; schema::LogSchema = CSV_LOG_SCHEMA)
    println(io, "# schema_version=$(schema.version)")
    println(io, "# schema_name=$(schema.name)")
    println(io, csv_header_line(schema))
    return nothing
end

############################
# Log sinks
############################

abstract type AbstractLogSink end

"""In-memory columnar log.

This keeps allocations low while still being easy to export to CSV.
"""
mutable struct SimLog <: AbstractLogSink
    t::Vector{Float64}
    time_us::Vector{UInt64}

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
        UInt64[],
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
    sizehint!(log.time_us, n)
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
    time_us::UInt64 = UInt64(round(Int, t * 1e6)),
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
    push!(sink.time_us, time_us)
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
    time_us::UInt64 = UInt64(round(Int, t * 1e6)),
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
        write_csv_header(io)
        sink.wrote_header = true
    end

    m = cmd.motors

    # Write a single row. printf-style formatting is used for floats to keep file size manageable.
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
        "%d,%d,%d,%d,%d,",
        nav_state,
        arming_state,
        mission_seq,
        mission_count,
        mission_finished
    )

    # lockstep clock
    print(io, time_us)
    print(io, "\n")

    return nothing
end

"""Write the full in-memory log to CSV.

This is mostly for convenience; for long runs, consider using `CSVLogSink`.
"""
function write_csv(log::SimLog, path::AbstractString)
    open(path, "w") do io
        write_csv_header(io)
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
                "%d,%d,%d,%d,%d,",
                log.nav_state[i],
                log.arming_state[i],
                log.mission_seq[i],
                log.mission_count[i],
                log.mission_finished[i]
            )

            # lockstep clock
            print(io, log.time_us[i])
            print(io, "\n")
        end
    end
    return nothing
end

end # module Logging
