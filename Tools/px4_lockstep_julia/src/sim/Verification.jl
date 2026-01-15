"""PX4Lockstep.Sim.Verification

Deterministic verification utilities and reference problems.

This module is intentionally **dependency-light** and focuses on problems that
exercise the exact numerical machinery used by the simulator:

* fixed-step Euler/RK4 stepping
* adaptive RK23/RK45 stepping
* `RigidBodyState` integration (pos/vel/quaternion/ω)

The primary purpose is to provide:

* fast unit tests that guard against numerical regressions
* runnable example scripts that quantify accuracy (analytic/invariant checks)
* a small set of reusable helpers for building reference solutions

Notes
-----
* All problems are expressed in terms of `RigidBodyState` so we can reuse the
  production integrators without introducing a separate generic ODE solver.
* These cases deliberately ignore PX4 and the full vehicle model; they're meant
  to isolate integrator and math correctness.
"""
module Verification

using LinearAlgebra
using StaticArrays

using ..Types
using ..RigidBody
using ..Integrators

export SHOCase,
    sho_analytic,
    sho_energy,
    sho_rhs,
    PendulumCase,
    pendulum_small_angle_analytic,
    pendulum_energy,
    pendulum_rhs,
    KeplerCircularCase,
    kepler_energy,
    kepler_angmom,
    kepler_rhs,
    kepler_circular_analytic,
    TorqueFreeRigidBodyCase,
    torque_free_rhs,
    rigidbody_rot_energy,
    rigidbody_angmom_body,
    rigidbody_angmom_ned,
    integrate_fixed

########################
# Shared helpers
########################

"""Integrate a `RigidBodyState` forward with a fixed step size.

This is a tiny convenience wrapper used by verification scripts/tests.

Arguments:
* `integrator`: `EulerIntegrator()` or `RK4Integrator()`
* `f`: RHS `f(t, x, u) -> RigidBodyDeriv`
* `x0`: initial state
* `dt`: fixed step size (s)
* `t_end`: final time (s)

Returns `(x_end, t_end_exact)`.
"""
function integrate_fixed(
    integrator::Union{EulerIntegrator,RK4Integrator},
    f,
    x0::RigidBodyState,
    dt::Float64,
    t_end::Float64;
    u = nothing,
)
    @assert dt > 0
    @assert t_end >= 0
    n = Int(round(t_end / dt))
    x = x0
    t = 0.0
    for _ = 1:n
        x = step_integrator(integrator, f, t, x, u, dt)
        t += dt
    end
    return x, t
end

########################
# Case: Simple harmonic oscillator (analytic)
########################

"""Simple harmonic oscillator embedded into `RigidBodyState`.

We store the scalar position in `pos_ned[1]` and velocity in `vel_ned[1]`.
All other components are held at zero.
"""
Base.@kwdef struct SHOCase
    ω0::Float64 = 2π
    x0::Float64 = 1.0
    v0::Float64 = 0.0
end

"""Analytic SHO solution.

Returns `(x(t), v(t))`.
"""
@inline function sho_analytic(c::SHOCase, t::Float64)
    ω = c.ω0
    x = c.x0 * cos(ω * t) + (c.v0 / ω) * sin(ω * t)
    v = -c.x0 * ω * sin(ω * t) + c.v0 * cos(ω * t)
    return x, v
end

"""SHO energy invariant: `E = 0.5*(v^2 + ω^2 x^2)`."""
@inline sho_energy(c::SHOCase, x::Float64, v::Float64) = 0.5 * (v * v + (c.ω0 * x)^2)

"""RHS for the SHO case in `RigidBodyState` form."""
function sho_rhs(c::SHOCase)
    ω2 = c.ω0 * c.ω0
    function f(t::Float64, s::RigidBodyState, _u)
        ax = -ω2 * s.pos_ned[1]
        return RigidBodyDeriv(
            pos_dot = s.vel_ned,
            vel_dot = vec3(ax, 0.0, 0.0),
            q_dot = Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = vec3(0.0, 0.0, 0.0),
        )
    end
    return f
end

########################
# Case: simple pendulum (nonlinear; small-angle analytic + energy invariant)
########################

"""Simple planar pendulum embedded into `RigidBodyState`.

We store the scalar angle `θ` in `pos_ned[1]` and angular rate `θ̇` in `vel_ned[1]`.

Dynamics:

    θ̈ + (g/L) sin(θ) = 0

Analytic solution exists in terms of elliptic integrals for general angles, but for
small angles the approximation reduces to SHO:

    θ(t) ≈ θ0 cos(ω t) + (θ̇0/ω) sin(ω t),  ω = √(g/L)

We use this as an analytic check in tests for small-angle initial conditions.
"""
Base.@kwdef struct PendulumCase
    g::Float64 = 9.80665
    L::Float64 = 1.0
    θ0::Float64 = 0.1
    θdot0::Float64 = 0.0
end

"""Small-angle analytic pendulum solution.

Returns `(θ(t), θ̇(t), ω)`.
"""
@inline function pendulum_small_angle_analytic(c::PendulumCase, t::Float64)
    ω = sqrt(c.g / c.L)
    θ = c.θ0 * cos(ω * t) + (c.θdot0 / ω) * sin(ω * t)
    θdot = -c.θ0 * ω * sin(ω * t) + c.θdot0 * cos(ω * t)
    return θ, θdot, ω
end

"""Pendulum energy per unit mass.

E = 0.5 (L θ̇)^2 + g L (1 - cos θ)

The constant factor `L` is included so the energy has units (m^2/s^2), but for
invariant checks only relative drift matters.
"""
@inline function pendulum_energy(c::PendulumCase, θ::Float64, θdot::Float64)
    return 0.5 * (c.L * θdot)^2 + c.g * c.L * (1.0 - cos(θ))
end

"""RHS for the nonlinear pendulum embedded in `RigidBodyState`."""
function pendulum_rhs(c::PendulumCase)
    @assert c.L > 0
    ω2 = c.g / c.L
    function f(t::Float64, s::RigidBodyState, _u)
        θ = s.pos_ned[1]
        θdot = s.vel_ned[1]
        θddot = -ω2 * sin(θ)
        return RigidBodyDeriv(
            pos_dot = vec3(θdot, 0.0, 0.0),
            vel_dot = vec3(θddot, 0.0, 0.0),
            q_dot = Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = vec3(0.0, 0.0, 0.0),
        )
    end
    return f
end

########################
# Case: 2-body circular orbit (analytic + invariants)
########################

"""Circular 2-body orbit (Kepler) embedded into `RigidBodyState`.

State mapping:
* `pos_ned` = position vector r
* `vel_ned` = velocity vector v

Analytic solution is valid when the initial conditions correspond to a circular orbit:
* r0 ⟂ v0
* |v0|^2 = μ/|r0|
"""
Base.@kwdef struct KeplerCircularCase
    μ::Float64 = 1.0
    r0::Vec3 = vec3(1.0, 0.0, 0.0)
    v0::Vec3 = vec3(0.0, 1.0, 0.0)

    # Validation tolerance for "is circular" checks.
    tol::Float64 = 1e-12
end

"""Kepler energy: ε = v^2/2 - μ/|r|."""
@inline function kepler_energy(μ::Float64, r::Vec3, v::Vec3)
    return 0.5 * dot(v, v) - μ / norm(r)
end

"""Kepler angular momentum vector: h = r × v."""
@inline kepler_angmom(r::Vec3, v::Vec3) = cross(r, v)

"""RHS for point-mass Kepler dynamics: r' = v, v' = -μ r / |r|^3."""
function kepler_rhs(μ::Float64)
    function f(t::Float64, s::RigidBodyState, _u)
        r = s.pos_ned
        v = s.vel_ned
        r2 = dot(r, r)
        r3 = r2 * sqrt(r2)
        a = (-μ / r3) * r
        return RigidBodyDeriv(
            pos_dot = v,
            vel_dot = a,
            q_dot = Quat(0.0, 0.0, 0.0, 0.0),
            ω_dot = vec3(0.0, 0.0, 0.0),
        )
    end
    return f
end

"""Analytic solution for a circular orbit.

Returns `(r(t), v(t), ω)`.

This computes the orbit plane basis from `(r0, v0)` and assumes the orbit is circular.
"""
function kepler_circular_analytic(c::KeplerCircularCase, t::Float64)
    μ = c.μ
    r0 = c.r0
    v0 = c.v0
    R = norm(r0)
    @assert R > 0

    # Circular constraints.
    @assert abs(dot(r0, v0)) <= c.tol "r0 must be perpendicular to v0 for a circular orbit"
    @assert abs(dot(v0, v0) - μ / R) <= max(c.tol, 1e-14) "|v0|^2 must equal μ/|r0| for a circular orbit"

    # Orthonormal basis for the orbit plane.
    e1 = r0 / R
    h = cross(r0, v0)
    hn = norm(h)
    @assert hn > 0
    e3 = h / hn
    e2 = cross(e3, e1)

    ω = sqrt(μ / (R^3))
    ct = cos(ω * t)
    st = sin(ω * t)

    r = R * (ct * e1 + st * e2)
    v = R * ω * (-st * e1 + ct * e2)
    return r, v, ω
end

########################
# Case: torque-free rigid body rotation (invariants)
########################

"""Torque-free rigid body with diagonal inertia embedded into `RigidBodyState`.

State mapping:
* `q_bn` is attitude (Body → NED)
* `ω_body` is body angular rate

No translation dynamics are integrated.
"""
Base.@kwdef struct TorqueFreeRigidBodyCase
    I_body::Vec3 = vec3(2.0, 1.0, 0.5)  # diagonal inertia (kg*m^2)
    ω0::Vec3 = vec3(1.0, 2.0, 3.0)
    q0::Quat = Quat(1.0, 0.0, 0.0, 0.0)
end

"""Rotational kinetic energy T = 0.5 ωᵀ I ω."""
@inline function rigidbody_rot_energy(I_body::Vec3, ω_body::Vec3)
    return 0.5 * (I_body[1] * ω_body[1]^2 + I_body[2] * ω_body[2]^2 + I_body[3] * ω_body[3]^2)
end

"""Angular momentum in body frame: L_body = I ⊙ ω."""
@inline rigidbody_angmom_body(I_body::Vec3, ω_body::Vec3) = Vec3(I_body[1] * ω_body[1], I_body[2] * ω_body[2], I_body[3] * ω_body[3])

"""Angular momentum in NED frame: L_ned = R_bn * L_body."""
@inline function rigidbody_angmom_ned(q_bn::Quat, I_body::Vec3, ω_body::Vec3)
    return quat_rotate(q_bn, rigidbody_angmom_body(I_body, ω_body))
end

"""RHS for torque-free rigid body rotation with diagonal inertia."""
function torque_free_rhs(c::TorqueFreeRigidBodyCase)
    Ix, Iy, Iz = c.I_body
    @assert Ix > 0 && Iy > 0 && Iz > 0
    function f(t::Float64, s::RigidBodyState, _u)
        ωx, ωy, ωz = s.ω_body
        ω̇x = ((Iy - Iz) / Ix) * ωy * ωz
        ω̇y = ((Iz - Ix) / Iy) * ωz * ωx
        ω̇z = ((Ix - Iy) / Iz) * ωx * ωy
        return RigidBodyDeriv(
            pos_dot = vec3(0.0, 0.0, 0.0),
            vel_dot = vec3(0.0, 0.0, 0.0),
            q_dot = quat_deriv(s.q_bn, s.ω_body),
            ω_dot = vec3(ω̇x, ω̇y, ω̇z),
        )
    end
    return f
end

end # module Verification
