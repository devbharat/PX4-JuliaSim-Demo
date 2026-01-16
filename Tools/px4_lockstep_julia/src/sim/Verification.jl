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
using ..Plant: PlantState
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
    integrate_fixed,
    Trajectory,
    simulate_trajectory,
    rk45_reference,
    resample_trajectory,
    rb_error,
    plant_error,
    error_series,
    invariant_series,
    compare_to_reference


########################
# Trajectory and comparison utilities (reference integration)
########################

"""A uniformly-sampled trajectory.

`x[k]` corresponds to time `t = (k-1)*dt`.

This representation avoids storing a Float64 time vector and makes it easy to
resample deterministically (integer index arithmetic).
"""
struct Trajectory{X}
    dt::Float64
    x::Vector{X}
end

@inline t_end(tr::Trajectory) = tr.dt * (length(tr.x) - 1)

"""Integrate a state forward and save it on a uniform output grid.

This is the core building block for reference/compare verification.

Arguments:
* `integrator`: any integrator supported by `step_integrator`
* `f`: RHS `f(t, x, u)`
* `x0`: initial state
* `dt`: output/sample interval (s). The integrator is asked to advance exactly `dt` each step.
* `t_end`: final time (s). Must be an exact multiple of `dt`.

Returns a `Trajectory` containing states at `t = 0:dt:t_end`.
"""
function simulate_trajectory(
    integrator,
    f,
    x0,
    dt::Float64,
    t_end::Float64;
    u = nothing,
)
    @assert dt > 0
    @assert t_end >= 0
    n = Int(round(t_end / dt))
    @assert isapprox(n * dt, t_end; atol = 0.0, rtol = 0.0) "t_end must be an exact multiple of dt"

    reset!(integrator)

    x = Vector{typeof(x0)}(undef, n + 1)
    x[1] = x0

    t = 0.0
    for k = 1:n
        x[k + 1] = step_integrator(integrator, f, t, x[k], u, dt)
        t += dt
    end
    return Trajectory{typeof(x0)}(dt, x)
end

"""Generate a high-accuracy RK45 reference trajectory.

This is intended as a numerical "truth" for cases without analytic solutions.

You typically pick `dt_ref` small enough that you can resample to the solver's
output grid exactly (e.g. `dt_ref = dt_test/10`).
"""
function rk45_reference(
    f,
    x0,
    dt_ref::Float64,
    t_end::Float64;
    u = nothing,
    # Default tolerances are intentionally tight.
    rtol_pos::Float64 = 1e-10,
    atol_pos::Float64 = 1e-12,
    rtol_vel::Float64 = 1e-10,
    atol_vel::Float64 = 1e-12,
    rtol_ω::Float64 = 1e-10,
    atol_ω::Float64 = 1e-12,
    atol_att_rad::Float64 = 1e-12,
    h_min::Float64 = 1e-9,
    h_max::Float64 = dt_ref,
    quantize_us::Bool = true,
    # Optional full-plant error control (disabled by default).
    plant_error_control::Bool = false,
    rtol_act::Float64 = 0.0,
    atol_act::Float64 = Inf,
    rtol_actdot::Float64 = 0.0,
    atol_actdot::Float64 = Inf,
    rtol_rotor::Float64 = 0.0,
    atol_rotor::Float64 = Inf,
    rtol_soc::Float64 = 0.0,
    atol_soc::Float64 = Inf,
    rtol_v1::Float64 = 0.0,
    atol_v1::Float64 = Inf,
)
    rk = RK45Integrator(
        rtol_pos = rtol_pos,
        atol_pos = atol_pos,
        rtol_vel = rtol_vel,
        atol_vel = atol_vel,
        rtol_ω = rtol_ω,
        atol_ω = atol_ω,
        atol_att_rad = atol_att_rad,
        h_min = h_min,
        h_max = h_max,
        quantize_us = quantize_us,
        plant_error_control = plant_error_control,
        rtol_act = rtol_act,
        atol_act = atol_act,
        rtol_actdot = rtol_actdot,
        atol_actdot = atol_actdot,
        rtol_rotor = rtol_rotor,
        atol_rotor = atol_rotor,
        rtol_soc = rtol_soc,
        atol_soc = atol_soc,
        rtol_v1 = rtol_v1,
        atol_v1 = atol_v1,
    )
    reset!(rk)
    return simulate_trajectory(rk, f, x0, dt_ref, t_end; u = u)
end

"""Resample a uniformly sampled trajectory by integer downsampling.

`dt_new` must be an exact integer multiple of `traj.dt`.
"""
function resample_trajectory(tr::Trajectory, dt_new::Float64)
    @assert dt_new > 0
    ratio = dt_new / tr.dt
    k = Int(round(ratio))
    @assert isapprox(k * tr.dt, dt_new; atol = 0.0, rtol = 0.0) "dt_new must be an exact integer multiple of dt"
    @assert k >= 1
    @assert ((length(tr.x) - 1) % k) == 0 "trajectory length must be an exact multiple of resample factor"
    x_new = tr.x[1:k:end]
    return Trajectory{eltype(tr.x)}(dt_new, collect(x_new))
end

"""Rigid-body state error components.

Returns a named tuple with:
* `pos` : ‖Δp‖ (m)
* `vel` : ‖Δv‖ (m/s)
* `att_rad` : SO(3) geodesic angle error (rad)
* `ω` : ‖Δω‖ (rad/s)
"""
@inline function rb_error(x::RigidBodyState, xref::RigidBodyState)
    pos = norm(x.pos_ned - xref.pos_ned)
    vel = norm(x.vel_ned - xref.vel_ned)
    ω = norm(x.ω_body - xref.ω_body)

    # Quaternion angle error: θ = 2 acos(|q⋅qref|)
    d = abs(dot(x.q_bn, xref.q_bn))
    d = clamp(d, -1.0, 1.0)
    att_rad = 2.0 * acos(d)
    return (pos = pos, vel = vel, att_rad = att_rad, ω = ω)
end

"""Plant-state error components.

This mirrors `rb_error` but extends it with a minimal set of non-rigid-body
continuous states that matter for the full-plant variable-step integrator:

* `rotor` : ‖Δrotor_ω‖ (rad/s)
* `soc`   : |ΔSOC|
* `v1`    : |ΔV1| (V)

Notes
-----
* Actuator states are not included by default; multirotor models typically capture
  the dominant lag in rotor ω dynamics.
* If actuator error terms are required, add a separate helper or extend this one.
"""
@inline function plant_error(x::PlantState{N}, xref::PlantState{N}) where {N}
    rb = rb_error(x.rb, xref.rb)
    rotor = norm(x.rotor_ω - xref.rotor_ω)
    soc = abs(x.batt_soc - xref.batt_soc)
    v1 = abs(x.batt_v1 - xref.batt_v1)
    return (pos = rb.pos, vel = rb.vel, att_rad = rb.att_rad, ω = rb.ω, rotor = rotor, soc = soc, v1 = v1)
end

"""Compute error series vs a reference trajectory.

Both trajectories must share the same uniform grid (`dt` and length).

Returns a named tuple:
* `t` : time vector (Float64)
* `pos_err`, `vel_err`, `att_err_rad`, `ω_err` : vectors
* `max` : (pos, vel, att_rad, ω)
* `rms` : (pos, vel, att_rad, ω)
"""
function error_series(ref::Trajectory{RigidBodyState}, sol::Trajectory{RigidBodyState})
    @assert isapprox(ref.dt, sol.dt; atol = 0.0, rtol = 0.0) "dt mismatch"
    @assert length(ref.x) == length(sol.x) "length mismatch"

    n = length(ref.x)
    pos_err = Vector{Float64}(undef, n)
    vel_err = Vector{Float64}(undef, n)
    att_err = Vector{Float64}(undef, n)
    ω_err = Vector{Float64}(undef, n)

    for i = 1:n
        e = rb_error(sol.x[i], ref.x[i])
        pos_err[i] = e.pos
        vel_err[i] = e.vel
        att_err[i] = e.att_rad
        ω_err[i] = e.ω
    end

    function _rms(v)
        s = 0.0
        @inbounds for x in v
            s += x * x
        end
        return sqrt(s / length(v))
    end

    max_nt = (
        pos = maximum(pos_err),
        vel = maximum(vel_err),
        att_rad = maximum(att_err),
        ω = maximum(ω_err),
    )
    rms_nt = (
        pos = _rms(pos_err),
        vel = _rms(vel_err),
        att_rad = _rms(att_err),
        ω = _rms(ω_err),
    )

    t = collect(0.0:ref.dt:(ref.dt * (n - 1)))
    return (
        t = t,
        pos_err = pos_err,
        vel_err = vel_err,
        att_err_rad = att_err,
        ω_err = ω_err,
        max = max_nt,
        rms = rms_nt,
    )
end

"""Compute error series vs a reference trajectory for `PlantState`.

Both trajectories must share the same uniform grid (`dt` and length).

Returns a named tuple:
* `t` : time vector (Float64)
* `pos_err`, `vel_err`, `att_err_rad`, `ω_err` : rigid-body error components
* `rotor_err` : ‖Δrotor_ω‖ (rad/s)
* `soc_err`   : |ΔSOC|
* `v1_err`    : |ΔV1| (V)
* `max` and `rms` summaries for all components.
"""
function error_series(ref::Trajectory{PlantState{N}}, sol::Trajectory{PlantState{N}}) where {N}
    @assert isapprox(ref.dt, sol.dt; atol = 0.0, rtol = 0.0) "dt mismatch"
    @assert length(ref.x) == length(sol.x) "length mismatch"

    n = length(ref.x)
    pos_err = Vector{Float64}(undef, n)
    vel_err = Vector{Float64}(undef, n)
    att_err = Vector{Float64}(undef, n)
    ω_err = Vector{Float64}(undef, n)
    rotor_err = Vector{Float64}(undef, n)
    soc_err = Vector{Float64}(undef, n)
    v1_err = Vector{Float64}(undef, n)

    for i = 1:n
        e = plant_error(sol.x[i], ref.x[i])
        pos_err[i] = e.pos
        vel_err[i] = e.vel
        att_err[i] = e.att_rad
        ω_err[i] = e.ω
        rotor_err[i] = e.rotor
        soc_err[i] = e.soc
        v1_err[i] = e.v1
    end

    function _rms(v)
        s = 0.0
        @inbounds for x in v
            s += x * x
        end
        return sqrt(s / length(v))
    end

    max_nt = (
        pos = maximum(pos_err),
        vel = maximum(vel_err),
        att_rad = maximum(att_err),
        ω = maximum(ω_err),
        rotor = maximum(rotor_err),
        soc = maximum(soc_err),
        v1 = maximum(v1_err),
    )
    rms_nt = (
        pos = _rms(pos_err),
        vel = _rms(vel_err),
        att_rad = _rms(att_err),
        ω = _rms(ω_err),
        rotor = _rms(rotor_err),
        soc = _rms(soc_err),
        v1 = _rms(v1_err),
    )

    t = collect(0.0:ref.dt:(ref.dt * (n - 1)))
    return (
        t = t,
        pos_err = pos_err,
        vel_err = vel_err,
        att_err_rad = att_err,
        ω_err = ω_err,
        rotor_err = rotor_err,
        soc_err = soc_err,
        v1_err = v1_err,
        max = max_nt,
        rms = rms_nt,
    )
end

"""Compute invariant time series and drift series for a trajectory.

`invfun(x) -> NamedTuple` should return scalar invariants (e.g., energy, |L|).

Returns a named tuple:
* `values` : NamedTuple of vectors (same keys as invfun)
* `drift`  : NamedTuple of vectors `values[k] - values[k][1]`
"""
function invariant_series(invfun, tr::Trajectory)
    inv0 = invfun(tr.x[1])
    ks = propertynames(inv0)
    n = length(tr.x)

    vals = ntuple(_ -> Vector{Float64}(undef, n), length(ks))
    for (j, k) in enumerate(ks)
        vals[j][1] = getproperty(inv0, k)
    end

    for i = 2:n
        inv = invfun(tr.x[i])
        for (j, k) in enumerate(ks)
            vals[j][i] = getproperty(inv, k)
        end
    end

    values_nt = NamedTuple{ks}(vals)

    drifts = ntuple(j -> begin
        v = values_nt[j]
        d = similar(v)
        v0 = v[1]
        @inbounds for i = 1:n
            d[i] = v[i] - v0
        end
        d
    end, length(ks))
    drift_nt = NamedTuple{ks}(drifts)
    return (values = values_nt, drift = drift_nt)
end

"""Compare a solver trajectory against a reference RK45 trajectory.

This helper is intended for verification scripts:
* computes error series vs time
* optionally computes invariant drift series for both solutions

If `invfun` is provided, it must accept `RigidBodyState` and return a `NamedTuple`.
"""
function compare_to_reference(ref::Trajectory{RigidBodyState}, sol::Trajectory{RigidBodyState}; invfun = nothing)
    err = error_series(ref, sol)
    if invfun === nothing
        return (err = err, invariants = nothing)
    end
    inv_ref = invariant_series(invfun, ref)
    inv_sol = invariant_series(invfun, sol)
    return (err = err, invariants = (ref = inv_ref, sol = inv_sol))
end

"""Compare a `PlantState` trajectory against a reference trajectory.

This mirrors `compare_to_reference(::Trajectory{RigidBodyState}, ...)` but uses
the `PlantState` error series (RB + rotor ω + SOC/V1).

If `invfun` is provided, it must accept `PlantState{N}` and return a `NamedTuple`
of scalar invariants. This enables "invariant drift" comparisons even for full-plant
systems (e.g., Kepler energy/angular momentum in the rigid-body subset).
"""
function compare_to_reference(ref::Trajectory{PlantState{N}}, sol::Trajectory{PlantState{N}}; invfun = nothing) where {N}
    err = error_series(ref, sol)
    if invfun === nothing
        return (err = err, invariants = nothing)
    end
    inv_ref = invariant_series(invfun, ref)
    inv_sol = invariant_series(invfun, sol)
    return (err = err, invariants = (ref = inv_ref, sol = inv_sol))
end

########################
# Shared helpers
########################

"""Integrate a `RigidBodyState` forward with a fixed step size.

This is a convenience wrapper used by verification scripts and tests.

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
    @assert isapprox(n * dt, t_end; atol = 0.0, rtol = 0.0) "t_end must be an exact multiple of dt for integrate_fixed"
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
