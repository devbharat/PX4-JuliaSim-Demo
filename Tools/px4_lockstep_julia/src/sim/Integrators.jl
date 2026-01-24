"""PX4Lockstep.Sim.Integrators

Fixed-step and adaptive (variable-step) ODE integrators used by the sim engine.

This module is intentionally lightweight and dependency-free (no DifferentialEquations.jl)
to preserve deterministic behavior, simplify review, and minimize dependencies.

Design:

* Integrators operate on `RigidBodyState` (rigid-body-only) or `PlantState` (full plant)
  with a dynamics function `f(t, x, u)` that returns the matching derivative type.
* Control inputs are treated as piecewise constant across a sim step. In closed-loop
  simulation this is the standard assumption: the controller produces a command at the
  start of the interval and the physical system evolves under that command until the next
  tick.

Design notes on solver choice emphasize time step stability; this file provides:
* Euler (debugging)
* RK4 (fixed-step default)
* RK23 (adaptive)
* RK45 / Dormand–Prince (adaptive)
"""
module Integrators

using ..RigidBody: RigidBodyState, RigidBodyDeriv, rb_add, rb_scale_add
using ..Types: Quat, quat_normalize
using ..Plant: PlantState, PlantDeriv, plant_add, plant_scale_add, plant_lincomb

export AbstractIntegrator,
    EulerIntegrator,
    RK4Integrator,
    RK23Integrator,
    RK45Integrator,
    IntegratorStats,
    last_stats,
    reset!,
    step_integrator

abstract type AbstractIntegrator end

"""Forward Euler integrator (1st order).

Useful for debugging but generally not recommended for production sims unless the step size
is very small.
"""
struct EulerIntegrator <: AbstractIntegrator end

"""Runge-Kutta 4th order integrator (fixed-step).

Good accuracy/compute tradeoff for flight dynamics and the common default in many aircraft
simulations.
"""
struct RK4Integrator <: AbstractIntegrator end

"""Lightweight statistics describing the most recent integration interval."""
Base.@kwdef struct IntegratorStats
    # Number of dynamics evaluations.
    nfev::Int = 0
    # Number of accepted substeps.
    naccept::Int = 0
    # Number of rejected substeps.
    nreject::Int = 0
    # Last accepted step size (s).
    h_last::Float64 = NaN
end

"""Return the last recorded integrator statistics (best-effort).

Fixed-step integrators return a default `IntegratorStats()`.
"""
last_stats(::AbstractIntegrator) = IntegratorStats()

"""Reset any internal step-size history in the integrator."""
reset!(::AbstractIntegrator) = nothing

############################
# Adaptive (variable-step) integrators
############################

"""Bogacki–Shampine RK3(2) adaptive integrator.

This is a relatively low-cost adaptive solver appropriate for many flight dynamics
workloads when high accuracy is not required.
"""
Base.@kwdef mutable struct RK23Integrator <: AbstractIntegrator
    # Tolerances by state group.
    rtol_pos::Float64 = 1e-6
    atol_pos::Float64 = 1e-6
    rtol_vel::Float64 = 1e-6
    atol_vel::Float64 = 1e-6
    rtol_ω::Float64 = 1e-6
    atol_ω::Float64 = 1e-6
    atol_att_rad::Float64 = 1e-6


    # If true, include non-rigid-body states (actuators, rotor ω, battery) in adaptive error control.
    # Default is false to preserve legacy behavior (RB-only error control).
    plant_error_control::Bool = false


    # Optional additional tolerances for full-plant adaptive stepping.
    #
    # By default these are set to Inf (ignored) to preserve legacy behavior where
    # adaptivity is driven only by rigid-body error. Enable by setting finite
    # absolute tolerances (and optional relative tolerances).
    rtol_act::Float64 = 0.0
    atol_act::Float64 = Inf
    rtol_actdot::Float64 = 0.0
    atol_actdot::Float64 = Inf
    rtol_rotor::Float64 = 0.0
    atol_rotor::Float64 = Inf
    rtol_soc::Float64 = 0.0
    atol_soc::Float64 = Inf
    rtol_v1::Float64 = 0.0
    atol_v1::Float64 = Inf

    # Step-size bounds.
    h_min::Float64 = 1e-6
    h_max::Float64 = Inf
    h_init::Float64 = 0.0
    max_substeps::Int = 10_000

    # Adaptation parameters.
    safety::Float64 = 0.9
    min_factor::Float64 = 0.2
    max_factor::Float64 = 5.0

    # If true, quantize substep sizes to integer microseconds.
    quantize_us::Bool = true

    # Internal state.
    last_h::Float64 = NaN
    last_stat::IntegratorStats = IntegratorStats()
end

"""Dormand–Prince RK5(4) adaptive integrator (a classic RK45 method)."""
Base.@kwdef mutable struct RK45Integrator <: AbstractIntegrator
    # Tolerances by state group.
    rtol_pos::Float64 = 1e-7
    atol_pos::Float64 = 1e-6
    rtol_vel::Float64 = 1e-7
    atol_vel::Float64 = 1e-6
    rtol_ω::Float64 = 1e-7
    atol_ω::Float64 = 1e-6
    atol_att_rad::Float64 = 1e-6


    # If true, include non-rigid-body states (actuators, rotor ω, battery) in adaptive error control.
    # Default is false to preserve legacy behavior (RB-only error control).
    plant_error_control::Bool = false


    # Optional additional tolerances for full-plant adaptive stepping.
    #
    # By default these are set to Inf (ignored) to preserve legacy behavior where
    # adaptivity is driven only by rigid-body error. Enable by setting finite
    # absolute tolerances (and optional relative tolerances).
    rtol_act::Float64 = 0.0
    atol_act::Float64 = Inf
    rtol_actdot::Float64 = 0.0
    atol_actdot::Float64 = Inf
    rtol_rotor::Float64 = 0.0
    atol_rotor::Float64 = Inf
    rtol_soc::Float64 = 0.0
    atol_soc::Float64 = Inf
    rtol_v1::Float64 = 0.0
    atol_v1::Float64 = Inf

    # Step-size bounds.
    h_min::Float64 = 1e-6
    h_max::Float64 = Inf
    h_init::Float64 = 0.0
    max_substeps::Int = 50_000

    # Adaptation parameters.
    safety::Float64 = 0.9
    min_factor::Float64 = 0.2
    max_factor::Float64 = 5.0

    # If true, quantize substep sizes to integer microseconds.
    quantize_us::Bool = true

    # Internal state.
    last_h::Float64 = NaN
    last_stat::IntegratorStats = IntegratorStats()
end

last_stats(i::Union{RK23Integrator,RK45Integrator}) = i.last_stat

function reset!(i::Union{RK23Integrator,RK45Integrator})
    i.last_h = NaN
    i.last_stat = IntegratorStats()
    return nothing
end

"""Advance state one step.

Arguments:
* `integrator` : any integrator supported by `step_integrator`
* `f`          : dynamics function `f(t, x, u)` returning `RigidBodyDeriv` or `PlantDeriv`
* `t`          : current time (s)
* `x`          : current state (`RigidBodyState` or `PlantState`)
* `u`          : control input (piecewise-constant over dt)
* `dt`         : step size (s)
"""
@inline function step_integrator(
    ::EulerIntegrator,
    f,
    t::Float64,
    x::RigidBodyState,
    u,
    dt::Float64,
)
    k1 = f(t, x, u)
    return rb_add(x, k1, dt)
end

@inline function step_integrator(
    ::RK4Integrator,
    f,
    t::Float64,
    x::RigidBodyState,
    u,
    dt::Float64,
)
    k1 = f(t, x, u)
    x2 = rb_add(x, k1, 0.5*dt)
    k2 = f(t + 0.5*dt, x2, u)
    x3 = rb_add(x, k2, 0.5*dt)
    k3 = f(t + 0.5*dt, x3, u)
    x4 = rb_add(x, k3, dt)
    k4 = f(t + dt, x4, u)
    return rb_scale_add(x, k1, k2, k3, k4, dt)
end

@inline function step_integrator(
    ::EulerIntegrator,
    f,
    t::Float64,
    x::PlantState{N,B},
    u,
    dt::Float64,
) where {N,B}
    k1 = f(t, x, u)
    return plant_add(x, k1, dt)
end

@inline function step_integrator(
    ::RK4Integrator,
    f,
    t::Float64,
    x::PlantState{N,B},
    u,
    dt::Float64,
) where {N,B}
    k1 = f(t, x, u)
    x2 = plant_add(x, k1, 0.5*dt)
    k2 = f(t + 0.5*dt, x2, u)
    x3 = plant_add(x, k2, 0.5*dt)
    k3 = f(t + 0.5*dt, x3, u)
    x4 = plant_add(x, k3, dt)
    k4 = f(t + dt, x4, u)
    return plant_scale_add(x, k1, k2, k3, k4, dt)
end

############################
# Adaptive solver helpers
############################

@inline function _quat_angle_error(q::Quat, q_ref::Quat)::Float64
    # Geodesic distance on SO(3), invariant to q ↦ -q.
    d = abs(sum(q .* q_ref))
    d = clamp(d, 0.0, 1.0)
    return 2.0 * acos(d)
end

@inline function _err_norm(
    x_hi::RigidBodyState,
    x_lo::RigidBodyState,
    x_ref::RigidBodyState,
    rtol_pos::Float64,
    atol_pos::Float64,
    rtol_vel::Float64,
    atol_vel::Float64,
    rtol_ω::Float64,
    atol_ω::Float64,
    atol_att_rad::Float64,
)::Float64
    # Position/velocity/angular-rate: component-wise scaled ∞-norm.
    Δp = x_hi.pos_ned - x_lo.pos_ned
    Δv = x_hi.vel_ned - x_lo.vel_ned
    Δω = x_hi.ω_body - x_lo.ω_body

    sp = atol_pos .+ rtol_pos .* max.(abs.(x_ref.pos_ned), abs.(x_hi.pos_ned))
    sv = atol_vel .+ rtol_vel .* max.(abs.(x_ref.vel_ned), abs.(x_hi.vel_ned))
    sω = atol_ω .+ rtol_ω .* max.(abs.(x_ref.ω_body), abs.(x_hi.ω_body))

    err_p = maximum(abs.(Δp) ./ sp)
    err_v = maximum(abs.(Δv) ./ sv)
    err_ω = maximum(abs.(Δω) ./ sω)

    # Attitude: scalar geodesic angle error.
    err_q = _quat_angle_error(x_hi.q_bn, x_lo.q_bn) / max(atol_att_rad, 1e-15)

    err = max(max(err_p, err_v), max(err_ω, err_q))
    return isfinite(err) ? err : Inf
end

@inline function _err_norm(
    x_hi::PlantState{N,B},
    x_lo::PlantState{N,B},
    x_ref::PlantState{N,B},
    rtol_pos::Float64,
    atol_pos::Float64,
    rtol_vel::Float64,
    atol_vel::Float64,
    rtol_ω::Float64,
    atol_ω::Float64,
    atol_att_rad::Float64,
)::Float64 where {N,B}
    # Legacy helper: adaptive error control based on rigid-body components only.
    # For full-plant error control, use `_err_norm(::RK23Integrator/RK45Integrator, ...)`.
    return _err_norm(
        x_hi.rb,
        x_lo.rb,
        x_ref.rb,
        rtol_pos,
        atol_pos,
        rtol_vel,
        atol_vel,
        rtol_ω,
        atol_ω,
        atol_att_rad,
    )
end

"""Adaptive error norm for `RigidBodyState` driven by an adaptive integrator's tolerances."""
@inline function _err_norm(
    i::Union{RK23Integrator,RK45Integrator},
    x_hi::RigidBodyState,
    x_lo::RigidBodyState,
    x_ref::RigidBodyState,
)::Float64
    return _err_norm(
        x_hi,
        x_lo,
        x_ref,
        i.rtol_pos,
        i.atol_pos,
        i.rtol_vel,
        i.atol_vel,
        i.rtol_ω,
        i.atol_ω,
        i.atol_att_rad,
    )
end

"""Adaptive error norm for full `PlantState`.

This extends rigid-body error control with optional additional state groups:
* actuator outputs (`motors_y`, `servos_y`)
* actuator rates (`motors_ydot`, `servos_ydot`)
* rotor speeds (`rotor_ω`)
* battery states (`power.soc`, `power.v1`)

To preserve legacy behavior, all of these are **ignored by default** because:
* `plant_error_control` defaults to `false`, and
* the corresponding absolute tolerances default to `Inf`.

Design intent:
* For full-plant accuracy, set finite `atol_*` (and optional `rtol_*`).
* For rigid-body accuracy (typical for many flight-control simulations), leave the defaults.
"""
@inline function _err_norm(
    i::Union{RK23Integrator,RK45Integrator},
    x_hi::PlantState{N,B},
    x_lo::PlantState{N,B},
    x_ref::PlantState{N,B},
)::Float64 where {N,B}
    # Always include rigid-body error.
    err = _err_norm(
        x_hi.rb,
        x_lo.rb,
        x_ref.rb,
        i.rtol_pos,
        i.atol_pos,
        i.rtol_vel,
        i.atol_vel,
        i.rtol_ω,
        i.atol_ω,
        i.atol_att_rad,
    )

    if !i.plant_error_control
        return err
    end

    # Actuator outputs.
    if isfinite(i.atol_act)
        # motors_y (12)
        @inbounds for k = 1:12
            Δ = x_hi.motors_y[k] - x_lo.motors_y[k]
            s = i.atol_act + i.rtol_act * max(abs(x_ref.motors_y[k]), abs(x_hi.motors_y[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
        # servos_y (8)
        @inbounds for k = 1:8
            Δ = x_hi.servos_y[k] - x_lo.servos_y[k]
            s = i.atol_act + i.rtol_act * max(abs(x_ref.servos_y[k]), abs(x_hi.servos_y[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
    end

    # Actuator rates.
    if isfinite(i.atol_actdot)
        @inbounds for k = 1:12
            Δ = x_hi.motors_ydot[k] - x_lo.motors_ydot[k]
            s =
                i.atol_actdot +
                i.rtol_actdot * max(abs(x_ref.motors_ydot[k]), abs(x_hi.motors_ydot[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
        @inbounds for k = 1:8
            Δ = x_hi.servos_ydot[k] - x_lo.servos_ydot[k]
            s =
                i.atol_actdot +
                i.rtol_actdot * max(abs(x_ref.servos_ydot[k]), abs(x_hi.servos_ydot[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
    end

    # Rotor speeds.
    if isfinite(i.atol_rotor)
        @inbounds for k = 1:N
            Δ = x_hi.rotor_ω[k] - x_lo.rotor_ω[k]
            s =
                i.atol_rotor +
                i.rtol_rotor * max(abs(x_ref.rotor_ω[k]), abs(x_hi.rotor_ω[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
    end

    # Battery (vectorized; B=1 matches legacy behavior).
    if isfinite(i.atol_soc)
        @inbounds for k = 1:B
            Δ = x_hi.power.soc[k] - x_lo.power.soc[k]
            s =
                i.atol_soc +
                i.rtol_soc * max(abs(x_ref.power.soc[k]), abs(x_hi.power.soc[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
    end

    if isfinite(i.atol_v1)
        @inbounds for k = 1:B
            Δ = x_hi.power.v1[k] - x_lo.power.v1[k]
            s = i.atol_v1 + i.rtol_v1 * max(abs(x_ref.power.v1[k]), abs(x_hi.power.v1[k]))
            s = max(s, 1e-15)
            err = max(err, abs(Δ) / s)
        end
    end

    return isfinite(err) ? err : Inf
end

@inline function _rb_lincomb(
    x::RigidBodyState,
    h::Float64,
    ks::NTuple{N,RigidBodyDeriv},
    as::NTuple{N,Float64},
) where {N}
    pos = x.pos_ned
    vel = x.vel_ned
    q = x.q_bn
    ω = x.ω_body
    @inbounds for i = 1:N
        w = as[i] * h
        pos = pos + ks[i].pos_dot * w
        vel = vel + ks[i].vel_dot * w
        q = q + ks[i].q_dot * w
        ω = ω + ks[i].ω_dot * w
    end
    return RigidBodyState(
        pos_ned = pos,
        vel_ned = vel,
        q_bn = quat_normalize(q),
        ω_body = ω,
    )
end

@inline function _clamp_step(i, h::Float64, remaining::Float64)
    # Clamp to remaining time and explicit bounds.
    h = min(h, remaining)
    h = min(h, i.h_max)
    # Allow the *final* substep to be smaller than h_min to avoid overshooting
    # the requested interval when `remaining < h_min`.
    if remaining >= i.h_min
        h = max(h, i.h_min)
    end

    if i.quantize_us
        # Quantize to integer microseconds.
        # (This improves repeatability and prevents floating point drift in long adaptive runs.)
        max_us = Int(round(min(i.h_max, remaining) * 1e6))
        max_us = max(max_us, 1)
        min_us = max(1, Int(round(i.h_min * 1e6)))
        min_us = min(min_us, max_us)
        h_us = Int(round(h * 1e6))
        h_us = clamp(h_us, min_us, max_us)
        h = h_us * 1e-6
    end
    return h
end

@inline function _snap_remaining(i, remaining::Float64)::Float64
    # When quantizing to microseconds, floating-point subtraction can leave a tiny positive
    # `remaining` (e.g., 5e-20) after an ostensibly exact final step. If the loop continues,
    # `_clamp_step` will quantize it to 1 µs and the interval would be overshot.
    #
    # Snap remaining to the nearest microsecond grid and clamp to [0, ∞).
    if i.quantize_us
        rem_us = Int(round(remaining * 1e6))
        rem_us = max(rem_us, 0)
        return rem_us * 1e-6
    end
    return remaining
end

@inline function _adapt_factor(
    err::Float64,
    order::Int,
    safety::Float64,
    minf::Float64,
    maxf::Float64,
)
    # Standard PI-free step adaptation: h_new = h * safety * err^(-1/order).
    if !(err > 0.0)  # err==0 or NaN
        return maxf
    end
    fac = safety * err^(-1.0 / Float64(order))
    return clamp(fac, minf, maxf)
end

############################
# RK23 (Bogacki–Shampine 3(2))
############################

function step_integrator(
    i::RK23Integrator,
    f,
    t::Float64,
    x0::RigidBodyState,
    u,
    dt::Float64,
)
    dt > 0 || return x0

    # Initial guess for substep size.
    h = if isfinite(i.last_h)
        i.last_h
    elseif i.h_init > 0.0
        i.h_init
    else
        dt
    end

    tcur = t
    x = x0
    remaining = dt

    nfev = 0
    naccept = 0
    nreject = 0
    h_last = NaN

    # Bogacki–Shampine coefficients.
    c2 = 0.5
    c3 = 0.75
    a21 = 0.5
    a32 = 0.75
    a41 = 2.0 / 9.0
    a42 = 1.0 / 3.0
    a43 = 4.0 / 9.0

    b1 = a41
    b2 = a42
    b3 = a43
    # b4 = 0

    b1h = 7.0 / 24.0
    b2h = 0.25
    b3h = 1.0 / 3.0
    b4h = 0.125

    while remaining > 0.0
        (naccept + nreject) < i.max_substeps ||
            error("RK23Integrator exceeded max_substeps=$(i.max_substeps)")
        h = _clamp_step(i, h, remaining)

        k1 = f(tcur, x, u);
        nfev += 1
        x2 = _rb_lincomb(x, h, (k1,), (a21,))
        k2 = f(tcur + c2*h, x2, u);
        nfev += 1

        x3 = _rb_lincomb(x, h, (k2,), (a32,))
        k3 = f(tcur + c3*h, x3, u);
        nfev += 1

        # 3rd order solution (high).
        x_hi = _rb_lincomb(x, h, (k1, k2, k3), (b1, b2, b3))
        k4 = f(tcur + h, x_hi, u);
        nfev += 1

        # 2nd order embedded solution (low).
        x_lo = _rb_lincomb(x, h, (k1, k2, k3, k4), (b1h, b2h, b3h, b4h))

        err = _err_norm(i, x_hi, x_lo, x)

        if err <= 1.0
            # Accept.
            x = x_hi
            tcur += h
            remaining -= h
            remaining = _snap_remaining(i, remaining)
            if i.quantize_us
                # Keep the stage time consistent with the snapped remaining.
                tcur = t + (dt - remaining)
            end
            naccept += 1
            h_last = h
            # Error estimate scales like O(h^3).
            h *= _adapt_factor(err, 3, i.safety, i.min_factor, i.max_factor)
        else
            # Reject and shrink step.
            nreject += 1
            h *= _adapt_factor(err, 3, i.safety, i.min_factor, i.max_factor)
            h = max(h, i.h_min)
            h <= i.h_min + 1e-18 &&
                error("RK23Integrator hit h_min=$(i.h_min) without meeting error tolerance")
        end
    end

    i.last_h = h
    i.last_stat =
        IntegratorStats(nfev = nfev, naccept = naccept, nreject = nreject, h_last = h_last)
    return x
end

############################
# RK45 (Dormand–Prince 5(4))
############################

function step_integrator(
    i::RK45Integrator,
    f,
    t::Float64,
    x0::RigidBodyState,
    u,
    dt::Float64,
)
    dt > 0 || return x0

    h = if isfinite(i.last_h)
        i.last_h
    elseif i.h_init > 0.0
        i.h_init
    else
        dt
    end

    tcur = t
    x = x0
    remaining = dt

    nfev = 0
    naccept = 0
    nreject = 0
    h_last = NaN

    # Dormand–Prince coefficients.
    c2 = 1.0 / 5.0
    c3 = 3.0 / 10.0
    c4 = 4.0 / 5.0
    c5 = 8.0 / 9.0
    c6 = 1.0

    a21 = 1.0 / 5.0

    a31 = 3.0 / 40.0
    a32 = 9.0 / 40.0

    a41 = 44.0 / 45.0
    a42 = -56.0 / 15.0
    a43 = 32.0 / 9.0

    a51 = 19372.0 / 6561.0
    a52 = -25360.0 / 2187.0
    a53 = 64448.0 / 6561.0
    a54 = -212.0 / 729.0

    a61 = 9017.0 / 3168.0
    a62 = -355.0 / 33.0
    a63 = 46732.0 / 5247.0
    a64 = 49.0 / 176.0
    a65 = -5103.0 / 18656.0

    # 5th order weights (also stage 7 state coefficients).
    b1 = 35.0 / 384.0
    b3 = 500.0 / 1113.0
    b4 = 125.0 / 192.0
    b5 = -2187.0 / 6784.0
    b6 = 11.0 / 84.0

    # 4th order embedded weights.
    b1h = 5179.0 / 57600.0
    b3h = 7571.0 / 16695.0
    b4h = 393.0 / 640.0
    b5h = -92097.0 / 339200.0
    b6h = 187.0 / 2100.0
    b7h = 1.0 / 40.0

    while remaining > 0.0
        (naccept + nreject) < i.max_substeps ||
            error("RK45Integrator exceeded max_substeps=$(i.max_substeps)")
        h = _clamp_step(i, h, remaining)

        k1 = f(tcur, x, u);
        nfev += 1
        x2 = _rb_lincomb(x, h, (k1,), (a21,))
        k2 = f(tcur + c2*h, x2, u);
        nfev += 1

        x3 = _rb_lincomb(x, h, (k1, k2), (a31, a32))
        k3 = f(tcur + c3*h, x3, u);
        nfev += 1

        x4 = _rb_lincomb(x, h, (k1, k2, k3), (a41, a42, a43))
        k4 = f(tcur + c4*h, x4, u);
        nfev += 1

        x5 = _rb_lincomb(x, h, (k1, k2, k3, k4), (a51, a52, a53, a54))
        k5 = f(tcur + c5*h, x5, u);
        nfev += 1

        x6 = _rb_lincomb(x, h, (k1, k2, k3, k4, k5), (a61, a62, a63, a64, a65))
        k6 = f(tcur + c6*h, x6, u);
        nfev += 1

        # 5th order solution.
        x_hi = _rb_lincomb(x, h, (k1, k3, k4, k5, k6), (b1, b3, b4, b5, b6))
        k7 = f(tcur + h, x_hi, u);
        nfev += 1

        # 4th order embedded solution.
        x_lo = _rb_lincomb(x, h, (k1, k3, k4, k5, k6, k7), (b1h, b3h, b4h, b5h, b6h, b7h))

        err = _err_norm(i, x_hi, x_lo, x)

        if err <= 1.0
            x = x_hi
            tcur += h
            remaining -= h
            remaining = _snap_remaining(i, remaining)
            if i.quantize_us
                # Keep the stage time consistent with the snapped remaining.
                tcur = t + (dt - remaining)
            end
            naccept += 1
            h_last = h
            # Error estimate scales like O(h^5).
            h *= _adapt_factor(err, 5, i.safety, i.min_factor, i.max_factor)
        else
            nreject += 1
            h *= _adapt_factor(err, 5, i.safety, i.min_factor, i.max_factor)
            h = max(h, i.h_min)
            h <= i.h_min + 1e-18 &&
                error("RK45Integrator hit h_min=$(i.h_min) without meeting error tolerance")
        end
    end

    i.last_h = h
    i.last_stat =
        IntegratorStats(nfev = nfev, naccept = naccept, nreject = nreject, h_last = h_last)
    return x
end


############################
# Adaptive solvers on PlantState (full-plant integration)
############################

"""RK23 adaptive step for `PlantState`.

Structurally identical to the `RigidBodyState` implementation, but integrates the full
plant state.

Implementation note:
* The default `_err_norm(::PlantState, ...)` delegates to the rigid-body error norm.
  This avoids the common failure mode where rotor ω dominates the scale and forces tiny
  steps. Once the coupled dynamics are implemented, `_err_norm` can be extended with
  rotor/battery/actuator tolerances.
"""
function step_integrator(
    i::RK23Integrator,
    f,
    t::Float64,
    x0::PlantState{N,B},
    u,
    dt::Float64,
) where {N,B}
    dt > 0 || return x0

    # Initial guess for substep size.
    h = if isfinite(i.last_h)
        i.last_h
    elseif i.h_init > 0.0
        i.h_init
    else
        dt
    end

    tcur = t
    x = x0
    remaining = dt

    nfev = 0
    naccept = 0
    nreject = 0
    h_last = NaN

    # Bogacki–Shampine coefficients.
    c2 = 0.5
    c3 = 0.75
    a21 = 0.5
    a32 = 0.75
    a41 = 2.0 / 9.0
    a42 = 1.0 / 3.0
    a43 = 4.0 / 9.0

    b1 = a41
    b2 = a42
    b3 = a43
    # b4 = 0

    b1h = 7.0 / 24.0
    b2h = 0.25
    b3h = 1.0 / 3.0
    b4h = 0.125

    while remaining > 0.0
        (naccept + nreject) < i.max_substeps ||
            error("RK23Integrator exceeded max_substeps=$(i.max_substeps)")
        h = _clamp_step(i, h, remaining)

        k1 = f(tcur, x, u);
        nfev += 1
        x2 = plant_lincomb(x, h, (k1,), (a21,))
        k2 = f(tcur + c2*h, x2, u);
        nfev += 1

        x3 = plant_lincomb(x, h, (k2,), (a32,))
        k3 = f(tcur + c3*h, x3, u);
        nfev += 1

        # 3rd order solution (high).
        x_hi = plant_lincomb(x, h, (k1, k2, k3), (b1, b2, b3))
        k4 = f(tcur + h, x_hi, u);
        nfev += 1

        # 2nd order embedded solution (low).
        x_lo = plant_lincomb(x, h, (k1, k2, k3, k4), (b1h, b2h, b3h, b4h))

        err = _err_norm(i, x_hi, x_lo, x)

        if err <= 1.0
            # Accept.
            x = x_hi
            tcur += h
            remaining -= h
            remaining = _snap_remaining(i, remaining)
            if i.quantize_us
                # Keep the stage time consistent with the snapped remaining.
                tcur = t + (dt - remaining)
            end
            naccept += 1
            h_last = h
            # Error estimate scales like O(h^3).
            h *= _adapt_factor(err, 3, i.safety, i.min_factor, i.max_factor)
        else
            # Reject and shrink step.
            nreject += 1
            h *= _adapt_factor(err, 3, i.safety, i.min_factor, i.max_factor)
            h = max(h, i.h_min)
            h <= i.h_min + 1e-18 &&
                error("RK23Integrator hit h_min=$(i.h_min) without meeting error tolerance")
        end
    end

    i.last_h = h
    i.last_stat =
        IntegratorStats(nfev = nfev, naccept = naccept, nreject = nreject, h_last = h_last)
    return x
end

"""RK45 adaptive step for `PlantState` (Dormand–Prince 5(4))."""
function step_integrator(
    i::RK45Integrator,
    f,
    t::Float64,
    x0::PlantState{N,B},
    u,
    dt::Float64,
) where {N,B}
    dt > 0 || return x0

    h = if isfinite(i.last_h)
        i.last_h
    elseif i.h_init > 0.0
        i.h_init
    else
        dt
    end

    tcur = t
    x = x0
    remaining = dt

    nfev = 0
    naccept = 0
    nreject = 0
    h_last = NaN

    # Dormand–Prince coefficients.
    c2 = 1.0 / 5.0
    c3 = 3.0 / 10.0
    c4 = 4.0 / 5.0
    c5 = 8.0 / 9.0
    c6 = 1.0

    a21 = 1.0 / 5.0

    a31 = 3.0 / 40.0
    a32 = 9.0 / 40.0

    a41 = 44.0 / 45.0
    a42 = -56.0 / 15.0
    a43 = 32.0 / 9.0

    a51 = 19372.0 / 6561.0
    a52 = -25360.0 / 2187.0
    a53 = 64448.0 / 6561.0
    a54 = -212.0 / 729.0

    a61 = 9017.0 / 3168.0
    a62 = -355.0 / 33.0
    a63 = 46732.0 / 5247.0
    a64 = 49.0 / 176.0
    a65 = -5103.0 / 18656.0

    # 5th order weights (also stage 7 state coefficients).
    b1 = 35.0 / 384.0
    b3 = 500.0 / 1113.0
    b4 = 125.0 / 192.0
    b5 = -2187.0 / 6784.0
    b6 = 11.0 / 84.0

    # 4th order embedded weights.
    b1h = 5179.0 / 57600.0
    b3h = 7571.0 / 16695.0
    b4h = 393.0 / 640.0
    b5h = -92097.0 / 339200.0
    b6h = 187.0 / 2100.0
    b7h = 1.0 / 40.0

    while remaining > 0.0
        (naccept + nreject) < i.max_substeps ||
            error("RK45Integrator exceeded max_substeps=$(i.max_substeps)")
        h = _clamp_step(i, h, remaining)

        k1 = f(tcur, x, u);
        nfev += 1
        x2 = plant_lincomb(x, h, (k1,), (a21,))
        k2 = f(tcur + c2*h, x2, u);
        nfev += 1

        x3 = plant_lincomb(x, h, (k1, k2), (a31, a32))
        k3 = f(tcur + c3*h, x3, u);
        nfev += 1

        x4 = plant_lincomb(x, h, (k1, k2, k3), (a41, a42, a43))
        k4 = f(tcur + c4*h, x4, u);
        nfev += 1

        x5 = plant_lincomb(x, h, (k1, k2, k3, k4), (a51, a52, a53, a54))
        k5 = f(tcur + c5*h, x5, u);
        nfev += 1

        x6 = plant_lincomb(x, h, (k1, k2, k3, k4, k5), (a61, a62, a63, a64, a65))
        k6 = f(tcur + c6*h, x6, u);
        nfev += 1

        # 5th order solution.
        x_hi = plant_lincomb(x, h, (k1, k3, k4, k5, k6), (b1, b3, b4, b5, b6))
        k7 = f(tcur + h, x_hi, u);
        nfev += 1

        # 4th order embedded solution.
        x_lo = plant_lincomb(x, h, (k1, k3, k4, k5, k6, k7), (b1h, b3h, b4h, b5h, b6h, b7h))

        err = _err_norm(i, x_hi, x_lo, x)

        if err <= 1.0
            x = x_hi
            tcur += h
            remaining -= h
            remaining = _snap_remaining(i, remaining)
            if i.quantize_us
                # Keep the stage time consistent with the snapped remaining.
                tcur = t + (dt - remaining)
            end
            naccept += 1
            h_last = h
            # Error estimate scales like O(h^5).
            h *= _adapt_factor(err, 5, i.safety, i.min_factor, i.max_factor)
        else
            nreject += 1
            h *= _adapt_factor(err, 5, i.safety, i.min_factor, i.max_factor)
            h = max(h, i.h_min)
            h <= i.h_min + 1e-18 &&
                error("RK45Integrator hit h_min=$(i.h_min) without meeting error tolerance")
        end
    end

    i.last_h = h
    i.last_stat =
        IntegratorStats(nfev = nfev, naccept = naccept, nreject = nreject, h_last = h_last)
    return x
end
end # module Integrators
