"""PX4Lockstep.Sim.Logging

Logging utilities.

The goal is to make it easy to:

* record a run deterministically
* export to CSV for quick plotting/analysis
* keep logging overhead reasonable

We avoid heavy dependencies (DataFrames, CSV.jl) to keep the core library lean. You can always convert the arrays into whatever tooling you prefer.
"""
module Logging

using ..Types: Vec3, Quat
using ..RigidBody: RigidBodyState
using Printf

export SimLog, log!, write_csv

"""A simple structured log.

Fields are vectors (time series). Extend this struct as you add new models.
"""
mutable struct SimLog
    t::Vector{Float64}
    pos_ned::Vector{NTuple{3,Float64}}
    pos_sp_ned::Vector{NTuple{3,Float64}}
    vel_ned::Vector{NTuple{3,Float64}}
    vel_sp_ned::Vector{NTuple{3,Float64}}
    acc_sp_ned::Vector{NTuple{3,Float64}}
    q_bn::Vector{NTuple{4,Float64}}
    ω_body::Vector{NTuple{3,Float64}}

    motor_cmd::Vector{NTuple{4,Float64}}

    nav_state::Vector{Int32}
    mission_seq::Vector{Int32}
    mission_count::Vector{Int32}
    mission_finished::Vector{Int32}
    yaw_sp::Vector{Float64}
    yawspeed_sp::Vector{Float64}
end

function SimLog()
    return SimLog(
        Float64[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{4,Float64}[],
        NTuple{3,Float64}[],
        NTuple{4,Float64}[],
        Int32[],
        Int32[],
        Int32[],
        Int32[],
        Float64[],
        Float64[],
    )
end

@inline function log!(log::SimLog, t::Float64, x::RigidBodyState, motor_cmd4::NTuple{4,Float64};
                      nav_state::Int32=Int32(-1), mission_seq::Int32=0, mission_count::Int32=0, mission_finished::Int32=0,
                      pos_sp::NTuple{3,Float64}=(NaN, NaN, NaN), vel_sp::NTuple{3,Float64}=(NaN, NaN, NaN),
                      acc_sp::NTuple{3,Float64}=(NaN, NaN, NaN), yaw_sp::Float64=NaN, yawspeed_sp::Float64=NaN)
    push!(log.t, t)
    push!(log.pos_ned, (x.pos_ned[1], x.pos_ned[2], x.pos_ned[3]))
    push!(log.pos_sp_ned, pos_sp)
    push!(log.vel_ned, (x.vel_ned[1], x.vel_ned[2], x.vel_ned[3]))
    push!(log.vel_sp_ned, vel_sp)
    push!(log.acc_sp_ned, acc_sp)
    push!(log.q_bn, (x.q_bn[1], x.q_bn[2], x.q_bn[3], x.q_bn[4]))
    push!(log.ω_body, (x.ω_body[1], x.ω_body[2], x.ω_body[3]))
    push!(log.motor_cmd, motor_cmd4)
    push!(log.nav_state, nav_state)
    push!(log.mission_seq, mission_seq)
    push!(log.mission_count, mission_count)
    push!(log.mission_finished, mission_finished)
    push!(log.yaw_sp, yaw_sp)
    push!(log.yawspeed_sp, yawspeed_sp)
    return nothing
end

"""Write a CSV log.

The schema is intentionally flat so it can be read by anything.
"""
function write_csv(log::SimLog, path::AbstractString)
    open(path, "w") do io
        println(io, "time_s,pos_x,pos_y,pos_z,pos_sp_x,pos_sp_y,pos_sp_z,vel_x,vel_y,vel_z,vel_sp_x,vel_sp_y,vel_sp_z,acc_sp_x,acc_sp_y,acc_sp_z,qw,qx,qy,qz,wx,wy,wz,m0,m1,m2,m3,nav_state,mission_seq,mission_count,mission_finished,yaw_sp,yawspeed_sp")
        n = length(log.t)
        for i in 1:n
            p = log.pos_ned[i]
            p_sp = log.pos_sp_ned[i]
            v = log.vel_ned[i]
            v_sp = log.vel_sp_ned[i]
            a_sp = log.acc_sp_ned[i]
            q = log.q_bn[i]
            w = log.ω_body[i]
            m = log.motor_cmd[i]
            @printf(io, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%.6f,%.6f\n",
                    log.t[i], p[1], p[2], p[3], p_sp[1], p_sp[2], p_sp[3],
                    v[1], v[2], v[3], v_sp[1], v_sp[2], v_sp[3], a_sp[1], a_sp[2], a_sp[3],
                    q[1], q[2], q[3], q[4], w[1], w[2], w[3],
                    m[1], m[2], m[3], m[4],
                    log.nav_state[i], log.mission_seq[i], log.mission_count[i], log.mission_finished[i],
                    log.yaw_sp[i], log.yawspeed_sp[i])
        end
    end
    return nothing
end

end # module Logging
