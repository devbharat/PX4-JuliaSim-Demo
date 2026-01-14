"""PX4Lockstep.Sim.Environment

World/environment models.

The goal is to keep these models composable:

* Atmosphere: density/pressure/temperature as a function of altitude (ISA1976 by default).
* Wind: mean wind + optional gust components.
* Gravity: constant or spherical gravity.

Terrain/contact is intentionally minimal right now (flat ground plane) and is modeled in
the vehicle layer, not here.
"""
module Environment

using ..Types: Vec3, vec3
using Random

export AbstractAtmosphere,
    ISA1976,
    air_temperature,
    air_pressure,
    air_density,
    AbstractWind,
    NoWind,
    ConstantWind,
    GustStep,
    OUWind,
    wind_velocity,
    step_wind!,
    set_mean_wind!,
    add_step_gust!,
    AbstractGravity,
    UniformGravity,
    SphericalGravity,
    gravity_accel,
    EnvironmentModel

############################
# Atmosphere
############################

abstract type AbstractAtmosphere end

"""International Standard Atmosphere 1976 (simplified up to 20km).

This implementation is sufficient for most low-altitude aircraft simulations and is kept
dependency-free.
"""
struct ISA1976 <: AbstractAtmosphere
    T0::Float64      # sea level temperature (K)
    p0::Float64      # sea level pressure (Pa)
    g0::Float64      # gravity at sea level (m/s^2)
    R::Float64       # specific gas constant for dry air (J/(kg*K))
end

ISA1976() = ISA1976(288.15, 101_325.0, 9.80665, 287.05287)

@inline function air_temperature(atm::ISA1976, alt_m::Float64)
    h = max(0.0, alt_m)
    if h <= 11_000.0
        L = -0.0065
        return atm.T0 + L*h
    else
        # Lower stratosphere is approximately isothermal.
        return 216.65
    end
end

@inline function air_pressure(atm::ISA1976, alt_m::Float64)
    h = max(0.0, alt_m)
    if h <= 11_000.0
        L = -0.0065
        T = atm.T0 + L*h
        return atm.p0 * (T/atm.T0)^(-atm.g0/(L*atm.R))
    else
        # p at 11km
        L = -0.0065
        T11 = atm.T0 + L*11_000.0
        p11 = atm.p0 * (T11/atm.T0)^(-atm.g0/(L*atm.R))
        T = 216.65
        return p11 * exp(-atm.g0*(h - 11_000.0)/(atm.R*T))
    end
end

@inline function air_density(atm::ISA1976, alt_m::Float64)
    T = air_temperature(atm, alt_m)
    p = air_pressure(atm, alt_m)
    return p / (atm.R*T)
end

############################
# Wind
############################

abstract type AbstractWind end

struct NoWind <: AbstractWind end
wind_velocity(::NoWind, ::Vec3, ::Float64) = vec3(0, 0, 0)

"""Advance a wind model by one step (default: no-op).

Stateful wind models (e.g. OU turbulence) should override this.
"""
step_wind!(::AbstractWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing
step_wind!(::NoWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing

"""Constant wind velocity in NED (m/s)."""
struct ConstantWind <: AbstractWind
    v_ned::Vec3
end
wind_velocity(w::ConstantWind, ::Vec3, ::Float64) = w.v_ned
step_wind!(::ConstantWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing

"""A simple step gust.

Adds `gust_v_ned` between `[t_on, t_off]`.

This is intentionally simple and deterministic; it can be replaced with Dryden/von Karman
models later.
"""
struct GustStep <: AbstractWind
    mean::AbstractWind
    gust_v_ned::Vec3
    t_on::Float64
    t_off::Float64
end

function wind_velocity(w::GustStep, pos_ned::Vec3, t::Float64)
    v = wind_velocity(w.mean, pos_ned, t)
    return (t >= w.t_on && t <= w.t_off) ? (v + w.gust_v_ned) : v
end

"""Advance the wrapped mean wind model.

`GustStep` is a purely *additive* wrapper. If the wrapped `mean` wind is stateful
(e.g. `OUWind`), it must still be stepped so the gust process evolves.
"""
function step_wind!(w::GustStep, pos_ned::Vec3, t::Float64, dt::Float64, rng::AbstractRNG)
    step_wind!(w.mean, pos_ned, t, dt, rng)
    return nothing
end

"""Ornstein–Uhlenbeck (OU) turbulence wind.

This produces a *stateful, deterministic* gust process when driven by a seeded RNG.

The OU process is a simple stochastic model with finite correlation time:

    dv = -(v/τ) dt + σ * sqrt(2/τ) * sqrt(dt) * ξ

where ξ ~ N(0, I). Stationary variance is σ^2.

This is not a full Dryden/von Karman model, but it is a practical baseline and easy to
extend.
"""
Base.@kwdef mutable struct OUWind <: AbstractWind
    mean::Vec3 = vec3(0, 0, 0)
    σ::Vec3 = vec3(0.0, 0.0, 0.0)
    τ_s::Float64 = 5.0
    v_gust::Vec3 = vec3(0.0, 0.0, 0.0)
    # Optional deterministic step gust for scenarios.
    step_gust::Vec3 = vec3(0.0, 0.0, 0.0)
    step_until_s::Float64 = -Inf
end

function wind_velocity(w::OUWind, ::Vec3, t::Float64)
    gust = (t <= w.step_until_s) ? w.step_gust : vec3(0.0, 0.0, 0.0)
    return w.mean + w.v_gust + gust
end

function step_wind!(w::OUWind, ::Vec3, ::Float64, dt::Float64, rng::AbstractRNG)
    # Exact discrete-time OU update (dt-invariant stationary variance).
    #
    # For each axis:
    #   v[k+1] = ϕ v[k] + σ * sqrt(1-ϕ^2) * ξ,   ξ ~ N(0,1)
    # where ϕ = exp(-dt/τ).
    if dt <= 0.0
        return nothing
    end

    τ = max(1e-6, w.τ_s)
    ϕ = exp(-dt / τ)
    scale = sqrt(max(0.0, 1.0 - ϕ^2))

    # Sample independent N(0,1) for each axis.
    w.v_gust =
        ϕ * w.v_gust + vec3(
            w.σ[1] * scale * randn(rng),
            w.σ[2] * scale * randn(rng),
            w.σ[3] * scale * randn(rng),
        )
    return nothing
end

"""Set the mean wind (mutable wind models only)."""
function set_mean_wind!(w::OUWind, v_ned::Vec3)
    w.mean = v_ned
    return nothing
end
set_mean_wind!(::AbstractWind, ::Vec3) =
    throw(ArgumentError("set_mean_wind! not supported for this wind type"))

"""Add a deterministic step gust for `duration_s` starting at the current time.

This is designed for scenario/event injection (e.g. a manual gust).
"""
function add_step_gust!(w::OUWind, dv_ned::Vec3, t_now::Float64, duration_s::Float64)
    w.step_gust = dv_ned
    w.step_until_s = t_now + max(0.0, duration_s)
    return nothing
end
add_step_gust!(::AbstractWind, ::Vec3, ::Float64, ::Float64) =
    throw(ArgumentError("add_step_gust! not supported for this wind type"))

############################
# Gravity
############################

abstract type AbstractGravity end

"""Uniform gravity (constant g) in NED."""
struct UniformGravity <: AbstractGravity
    g::Float64
end

gravity_accel(g::UniformGravity, ::Vec3, ::Float64) = vec3(0.0, 0.0, g.g)

"""Spherical gravity `μ / r^2`.

This model expects `r` to be measured from Earth's center. For NED local frames, it is
usually adequate to use `UniformGravity`; use `SphericalGravity` for high-altitude or
long-range scenarios.
"""
struct SphericalGravity <: AbstractGravity
    μ::Float64      # [m^3/s^2]
    r0::Float64     # reference radius (m)
end

function gravity_accel(g::SphericalGravity, pos_ned::Vec3, ::Float64)
    # Approximate: treat local down as radial. Use r = r0 - z (since z is down).
    r = max(1.0, g.r0 - pos_ned[3])
    return vec3(0.0, 0.0, g.μ/(r*r))
end

############################
# Environment container
############################

"""A composable environment model.

`origin_alt_msl_m` defines the mean-sea-level (MSL) altitude of the local NED origin.

This matters for atmosphere models: in a typical PX4 local frame, `pos_ned = (0,0,0)`
corresponds to the *home* location, not sea level. Using `origin_alt_msl_m` keeps the
physics (density) consistent with the lat/lon/alt fed to PX4.
"""
struct EnvironmentModel{A<:AbstractAtmosphere,W<:AbstractWind,G<:AbstractGravity}
    atmosphere::A
    wind::W
    gravity::G
    origin_alt_msl_m::Float64
end

EnvironmentModel(;
    atmosphere::AbstractAtmosphere = ISA1976(),
    wind::AbstractWind = NoWind(),
    gravity::AbstractGravity = UniformGravity(9.80665),
    origin_alt_msl_m::Float64 = 0.0,
) = EnvironmentModel(atmosphere, wind, gravity, origin_alt_msl_m)

end # module Environment
