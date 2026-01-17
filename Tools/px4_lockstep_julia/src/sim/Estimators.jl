"""PX4Lockstep.Sim.Estimators

Truth → estimated state models.

You said EKF2 will not be in the loop for now, but PX4 still expects "state estimate"
topics (local position, attitude, body rates, etc.). This module provides a clean place
to inject:
- noise
- slowly varying bias
- fixed delay / sample-and-hold effects

without polluting the vehicle dynamics or the PX4 bridge.
"""
module Estimators

using ..Types: Vec3, Quat, vec3, quat_mul, quat_normalize, quat_from_axis_angle
using ..RigidBody: RigidBodyState
using ..Noise: gaussian, gaussian_vec, AR1, AR1Vec
import ..Noise: step!, reset!
using Random

export EstimatedState,
    AbstractEstimator, TruthEstimator, NoisyEstimator, DelayedEstimator, estimate!, reset!

"""Estimated state used to drive the autopilot interface."""
Base.@kwdef struct EstimatedState
    pos_ned::Vec3
    vel_ned::Vec3
    q_bn::Quat
    ω_body::Vec3
end

abstract type AbstractEstimator end

"""Estimator that returns truth exactly."""
struct TruthEstimator <: AbstractEstimator end

@inline function reset!(::TruthEstimator)
    return nothing
end

@inline function estimate!(
    ::TruthEstimator,
    ::AbstractRNG,
    t::Float64,
    x::RigidBodyState,
    dt_hint::Float64,
)
    return EstimatedState(
        pos_ned = x.pos_ned,
        vel_ned = x.vel_ned,
        q_bn = x.q_bn,
        ω_body = x.ω_body,
    )
end

"""Noisy estimator with AR(1) bias and additive Gaussian noise.

Noise is injected into:
- local position (NED)
- local velocity (NED)
- yaw (rotation about NED +Z axis)
- body rates

This is intended to approximate "EKF output" error statistics, not raw sensors.
"""
mutable struct NoisyEstimator <: AbstractEstimator
    pos_sigma_m::Vec3
    vel_sigma_mps::Vec3
    yaw_sigma_rad::Float64
    rate_sigma_rad_s::Vec3

    # Bias processes (shared time constant by default; tune as needed).
    pos_bias::AR1Vec
    vel_bias::AR1Vec
    yaw_bias::AR1
    rate_bias::AR1Vec
end

function NoisyEstimator(;
    pos_sigma_m::Vec3 = vec3(0.0, 0.0, 0.0),
    vel_sigma_mps::Vec3 = vec3(0.0, 0.0, 0.0),
    yaw_sigma_rad::Float64 = 0.0,
    rate_sigma_rad_s::Vec3 = vec3(0.0, 0.0, 0.0),
    bias_tau_s::Float64 = Inf,
    pos_bias_sigma_m::Vec3 = vec3(0.0, 0.0, 0.0),
    vel_bias_sigma_mps::Vec3 = vec3(0.0, 0.0, 0.0),
    yaw_bias_sigma_rad::Float64 = 0.0,
    rate_bias_sigma_rad_s::Vec3 = vec3(0.0, 0.0, 0.0),
)
    return NoisyEstimator(
        pos_sigma_m,
        vel_sigma_mps,
        yaw_sigma_rad,
        rate_sigma_rad_s,
        AR1Vec(bias_tau_s, pos_bias_sigma_m),
        AR1Vec(bias_tau_s, vel_bias_sigma_mps),
        AR1(bias_tau_s, yaw_bias_sigma_rad),
        AR1Vec(bias_tau_s, rate_bias_sigma_rad_s),
    )
end

function reset!(e::NoisyEstimator)
    reset!(e.pos_bias)
    reset!(e.vel_bias)
    reset!(e.yaw_bias)
    reset!(e.rate_bias)
    return e
end

function estimate!(
    e::NoisyEstimator,
    rng::AbstractRNG,
    t::Float64,
    x::RigidBodyState,
    dt_hint::Float64,
)
    # IMPORTANT: keep estimator stepping on an exact cadence.
    #
    # Using (t - last_t) reintroduces float subtraction noise which can change
    # bias evolution over long horizons across platforms/libm versions.
    dt_hint > 0.0 || error("dt_hint must be > 0")
    dt = dt_hint

    pb = step!(e.pos_bias, rng, dt)
    vb = step!(e.vel_bias, rng, dt)
    yb = step!(e.yaw_bias, rng, dt)
    rb = step!(e.rate_bias, rng, dt)

    pos = x.pos_ned + pb + gaussian_vec(rng, e.pos_sigma_m)
    vel = x.vel_ned + vb + gaussian_vec(rng, e.vel_sigma_mps)
    ω = x.ω_body + rb + gaussian_vec(rng, e.rate_sigma_rad_s)

    # Apply yaw bias/noise as a rotation about NED Z.
    dyaw = yb + gaussian(rng, e.yaw_sigma_rad)
    q = x.q_bn
    if dyaw != 0.0
        qz = quat_from_axis_angle(vec3(0.0, 0.0, 1.0), dyaw)
        q = quat_normalize(quat_mul(qz, q))
    end

    return EstimatedState(pos_ned = pos, vel_ned = vel, q_bn = q, ω_body = ω)
end

"""Fixed-step delay wrapper for an estimator.

This is intentionally quantized in steps (not continuous delay) for deterministic
lockstep simulation. Use a delay that is a multiple of the autopilot update period.
"""
mutable struct DelayedEstimator{E<:AbstractEstimator} <: AbstractEstimator
    inner::E
    dt_est_us::Int
    delay_steps::Int
    ring::Vector{EstimatedState}
    idx::Int
    filled::Int
end

@inline function _to_us_strict(t_s::Float64, name::AbstractString)
    t_s >= 0.0 || error("$name must be >= 0")
    t_us_f = t_s * 1e6
    t_us = Int(round(t_us_f))
    # Enforce microsecond quantization to keep delay behavior deterministic.
    if abs(t_us_f - t_us) > 1e-6
        error("$name must be microsecond-quantized (got $t_s s)")
    end
    return t_us
end

function DelayedEstimator(
    inner::E;
    delay_s::Float64 = 0.0,
    dt_est::Float64 = 0.002,
) where {E<:AbstractEstimator}
    dt_us = _to_us_strict(dt_est, "dt_est")
    dt_us > 0 || error("dt_est must be > 0")
    delay_us = _to_us_strict(delay_s, "delay_s")
    if delay_us % dt_us != 0
        error(
            "delay_s must be an exact multiple of dt_est (delay_s=$delay_s, dt_est=$dt_est)",
        )
    end
    delay_steps = delay_us ÷ dt_us
    ring_len = max(1, delay_steps + 1)
    ring = [
        EstimatedState(
            pos_ned = vec3(0.0, 0.0, 0.0),
            vel_ned = vec3(0.0, 0.0, 0.0),
            q_bn = Quat(1.0, 0.0, 0.0, 0.0),
            ω_body = vec3(0.0, 0.0, 0.0),
        ) for _ = 1:ring_len
    ]
    return DelayedEstimator(inner, dt_us, delay_steps, ring, 1, 0)
end

function reset!(e::DelayedEstimator)
    e.idx = 1
    e.filled = 0
    reset!(e.inner)
    return e
end

function estimate!(
    e::DelayedEstimator,
    rng::AbstractRNG,
    t::Float64,
    x::RigidBodyState,
    dt_hint::Float64,
)
    # DelayedEstimator is step-quantized for determinism. If the stepping cadence
    # doesn't match the cadence used to compute `delay_steps`, the configured delay
    # becomes wrong (silently). Fail fast.
    dt_hint_us = _to_us_strict(dt_hint, "dt_hint")
    if dt_hint_us != e.dt_est_us
        error(
            "DelayedEstimator dt mismatch: dt_hint=$(dt_hint) s ($(dt_hint_us) μs) != configured dt_est=$(e.dt_est_us * 1e-6) s ($(e.dt_est_us) μs).",
        )
    end
    cur = estimate!(e.inner, rng, t, x, dt_hint)
    e.ring[e.idx] = cur
    e.filled = min(length(e.ring), e.filled + 1)

    # Delayed index relative to the most recently written slot.
    di = e.idx - e.delay_steps
    if di <= 0
        di += length(e.ring)
    end

    out = (e.filled > e.delay_steps) ? e.ring[di] : cur

    e.idx += 1
    if e.idx > length(e.ring)
        e.idx = 1
    end

    return out
end

end # module Estimators
