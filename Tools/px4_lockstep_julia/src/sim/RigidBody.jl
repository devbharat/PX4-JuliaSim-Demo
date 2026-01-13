"""PX4Lockstep.Sim.RigidBody

Core rigid-body state representation and ODE helper operations.

This file is intentionally *model agnostic*: it defines how we represent and integrate
6DOF rigid body state. Specific aircraft models compute forces and moments.
"""
module RigidBody

using ..Types: Vec3, Quat, vec3, quat_normalize, quat_mul
using LinearAlgebra
using StaticArrays

export RigidBodyState, RigidBodyDeriv,
       rb_zero, rb_deriv_zero,
       rb_add, rb_scale_add,
       quat_deriv

"""6DOF rigid-body state.

Fields:
* `pos_ned` : position in NED (m)
* `vel_ned` : velocity in NED (m/s)
* `q_bn`    : quaternion body→NED (w,x,y,z)
* `ω_body`  : body rates (p,q,r) in rad/s
"""
Base.@kwdef struct RigidBodyState
    pos_ned::Vec3 = vec3(0, 0, 0)
    vel_ned::Vec3 = vec3(0, 0, 0)
    q_bn::Quat = Quat(1.0, 0.0, 0.0, 0.0)
    ω_body::Vec3 = vec3(0, 0, 0)
end

"""Time derivative of `RigidBodyState`."""
Base.@kwdef struct RigidBodyDeriv
    pos_dot::Vec3 = vec3(0, 0, 0)
    vel_dot::Vec3 = vec3(0, 0, 0)
    q_dot::Quat = Quat(0.0, 0.0, 0.0, 0.0)
    ω_dot::Vec3 = vec3(0, 0, 0)
end

rb_zero() = RigidBodyState()
rb_deriv_zero() = RigidBodyDeriv()

"""Quaternion derivative `q̇ = 0.5 * q ⊗ ω_quat`.

`q` is body→NED, and `ω_body` is body rates.
"""
@inline function quat_deriv(q::Quat, ω_body::Vec3)
    ωq = Quat(0.0, ω_body[1], ω_body[2], ω_body[3])
    return 0.5 .* quat_mul(q, ωq)
end

"""Return a new state: `x + scale * xdot`.

We normalize the quaternion to avoid drift during multi-stage integration.
"""
@inline function rb_add(x::RigidBodyState, xdot::RigidBodyDeriv, scale::Float64)
    return RigidBodyState(
        pos_ned = x.pos_ned + xdot.pos_dot * scale,
        vel_ned = x.vel_ned + xdot.vel_dot * scale,
        q_bn    = quat_normalize(x.q_bn + xdot.q_dot * scale),
        ω_body  = x.ω_body + xdot.ω_dot * scale,
    )
end

"""In-place style helper returning the state update: `x + (k1 + 2k2 + 2k3 + k4) * (dt/6)`.

Used by RK4 integrator.
"""
@inline function rb_scale_add(x::RigidBodyState, k1::RigidBodyDeriv, k2::RigidBodyDeriv,
                              k3::RigidBodyDeriv, k4::RigidBodyDeriv, dt::Float64)
    w = dt / 6.0
    pos_dot = (k1.pos_dot + 2.0*k2.pos_dot + 2.0*k3.pos_dot + k4.pos_dot)
    vel_dot = (k1.vel_dot + 2.0*k2.vel_dot + 2.0*k3.vel_dot + k4.vel_dot)
    q_dot   = (k1.q_dot   + 2.0*k2.q_dot   + 2.0*k3.q_dot   + k4.q_dot)
    ω_dot   = (k1.ω_dot   + 2.0*k2.ω_dot   + 2.0*k3.ω_dot   + k4.ω_dot)
    return RigidBodyState(
        pos_ned = x.pos_ned + pos_dot * w,
        vel_ned = x.vel_ned + vel_dot * w,
        q_bn    = quat_normalize(x.q_bn + q_dot * w),
        ω_body  = x.ω_body + ω_dot * w,
    )
end

end # module RigidBody
