"""PX4Lockstep.Sim.Noise

Deterministic noise/bias helpers.

The goal is to make randomness:
- explicit
- reproducible (seeded)
- injectable at clear boundaries (e.g. estimator outputs, wind disturbances)

This module is intentionally small and dependency-free.
"""
module Noise

using ..Types: Vec3, vec3
using Random

export gaussian, gaussian_vec, AR1, AR1Vec, step!, reset!

@inline gaussian(rng::AbstractRNG, σ::Float64) = (σ == 0.0 ? 0.0 : σ * randn(rng))

@inline function gaussian_vec(rng::AbstractRNG, σ::Vec3)
    return vec3(σ[1] * randn(rng), σ[2] * randn(rng), σ[3] * randn(rng))
end

"""Scalar AR(1) bias process with approximate time constant.

A continuous-time Ornstein-Uhlenbeck process discretized via:
  x[k+1] = ϕ x[k] + σ * sqrt(1-ϕ^2) * w[k],  w ~ N(0,1)
where ϕ = exp(-dt/τ).
"""
mutable struct AR1
    tau_s::Float64
    sigma::Float64
    x::Float64
end

AR1(tau_s::Float64, sigma::Float64) = AR1(tau_s, sigma, 0.0)

@inline function reset!(a::AR1)
    a.x = 0.0
    return a
end

@inline function step!(a::AR1, rng::AbstractRNG, dt::Float64)
    if a.sigma == 0.0 || !isfinite(a.tau_s) || a.tau_s <= 0.0 || dt <= 0.0
        return a.x
    end
    ϕ = exp(-dt / a.tau_s)
    a.x = ϕ * a.x + a.sigma * sqrt(max(0.0, 1.0 - ϕ^2)) * randn(rng)
    return a.x
end

"""3-axis AR(1) bias process."""
mutable struct AR1Vec
    tau_s::Float64
    sigma::Vec3
    x::Vec3
end

AR1Vec(tau_s::Float64, sigma::Vec3) = AR1Vec(tau_s, sigma, vec3(0.0, 0.0, 0.0))

@inline function reset!(a::AR1Vec)
    a.x = vec3(0.0, 0.0, 0.0)
    return a
end

@inline function step!(a::AR1Vec, rng::AbstractRNG, dt::Float64)
    if (!isfinite(a.tau_s) || a.tau_s <= 0.0 || dt <= 0.0) ||
       (a.sigma[1] == 0.0 && a.sigma[2] == 0.0 && a.sigma[3] == 0.0)
        return a.x
    end
    ϕ = exp(-dt / a.tau_s)
    scale = sqrt(max(0.0, 1.0 - ϕ^2))
    a.x =
        ϕ * a.x + vec3(
            a.sigma[1]*scale*randn(rng),
            a.sigma[2]*scale*randn(rng),
            a.sigma[3]*scale*randn(rng),
        )
    return a.x
end

end # module Noise
