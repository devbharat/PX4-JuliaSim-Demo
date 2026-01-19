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

using ..Types: Vec3, WorldOrigin, vec3
using Random

export AbstractAtmosphere,
    ISA1976,
    air_temperature,
    air_pressure,
    air_density,
    WorldOrigin,
    AbstractWind,
    NoWind,
    ConstantWind,
    SampledWind,
    GustStep,
    OUWind,
    wind_velocity,
    step_wind!,
    sample_wind!,
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

"""Convert seconds to integer microseconds with an exactness check.

This is used for event-aligned wind constructs (gust start/stop) so activation is
evaluated on the same microsecond grid as the canonical runtime engine.
"""
@inline function _sec_to_us(t_s::Real; allow_zero::Bool = true)::UInt64
    t = Float64(t_s)
    t >= 0.0 || throw(ArgumentError("time must be >= 0 s; got $t"))
    us = round(Int64, t * 1e6)
    # Require exact microsecond representability (same spirit as Runtime.dt_to_us).
    abs(t - (Float64(us) * 1e-6)) <= 1e-15 || throw(
        ArgumentError(
            "time=$t is not an integer multiple of 1 µs (got us=$us -> $(Float64(us)*1e-6))",
        ),
    )
    if !allow_zero && us < 1
        throw(ArgumentError("time must be >= 1 µs"))
    end
    return UInt64(max(0, us))
end

@inline _round_us(t_s::Float64)::UInt64 = UInt64(round(Int64, t_s * 1e6))

struct NoWind <: AbstractWind end
wind_velocity(::NoWind, ::Vec3, ::Float64) = vec3(0, 0, 0)
"""Advance a wind model by one step (default: no-op).

Stateful wind models (e.g. OU turbulence) should override this.
"""
step_wind!(::AbstractWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing
step_wind!(::NoWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing

# -----------------------------------------------------------------------------
# Integer-microsecond overloads (canonical engine timebase)
# -----------------------------------------------------------------------------

"""Wind velocity sampled at integer-microsecond time `t_us`.

The canonical simulation engine uses integer microseconds for all scheduling. These
overloads make it possible to express wind constructs (e.g. gust start/stop) without
Float64 time edge cases.
"""
@inline function wind_velocity(w::AbstractWind, pos_ned::Vec3, t_us::UInt64)
    return wind_velocity(w, pos_ned, Float64(t_us) * 1e-6)
end

"""Advance a wind model by one step using integer microseconds."""
@inline function step_wind!(
    w::AbstractWind,
    pos_ned::Vec3,
    t_us::UInt64,
    dt_us::UInt64,
    rng::AbstractRNG,
)
    return step_wind!(w, pos_ned, Float64(t_us) * 1e-6, Float64(dt_us) * 1e-6, rng)
end

"""Sample wind at integer-microsecond time (default: direct call to `wind_velocity`)."""
@inline function sample_wind!(w::AbstractWind, pos_ned::Vec3, t_us::UInt64)
    return wind_velocity(w, pos_ned, t_us)
end

"""Constant wind velocity in NED (m/s)."""
struct ConstantWind <: AbstractWind
    v_ned::Vec3
end
wind_velocity(w::ConstantWind, ::Vec3, ::Float64) = w.v_ned
step_wind!(::ConstantWind, ::Vec3, ::Float64, ::Float64, ::AbstractRNG) = nothing

"""Sample-and-hold wrapper for a wind model.

`SampledWind` holds the most recent sampled wind value and returns it from
`wind_velocity(...)` regardless of time/position. This keeps wind forcing constant
across RK4 stages within a physics tick.
"""
mutable struct SampledWind{W<:AbstractWind} <: AbstractWind
    inner::W
    sample_ned::Vec3
end

SampledWind(inner::W) where {W<:AbstractWind} = SampledWind(inner, vec3(0.0, 0.0, 0.0))

function wind_velocity(w::SampledWind, ::Vec3, ::Float64)
    return w.sample_ned
end

function wind_velocity(w::SampledWind, ::Vec3, ::UInt64)
    return w.sample_ned
end

function step_wind!(
    w::SampledWind,
    pos_ned::Vec3,
    t::Float64,
    dt::Float64,
    rng::AbstractRNG,
)
    step_wind!(w.inner, pos_ned, t, dt, rng)
    return nothing
end

"""Sample the wrapped wind model and hold the value for the current tick."""
function sample_wind!(w::SampledWind, pos_ned::Vec3, t::Float64)
    return sample_wind!(w, pos_ned, _round_us(t))
end

function sample_wind!(w::SampledWind, pos_ned::Vec3, t_us::UInt64)
    w.sample_ned = wind_velocity(w.inner, pos_ned, t_us)
    return w.sample_ned
end

sample_wind!(w::AbstractWind, pos_ned::Vec3, t::Float64) = wind_velocity(w, pos_ned, t)

"""A simple step gust.

Adds `gust_v_ned` between `[t_on, t_off)`.

This is intentionally simple and deterministic; it can be replaced with Dryden/von Karman
models later.
"""
struct GustStep <: AbstractWind
    mean::AbstractWind
    gust_v_ned::Vec3
    t_on_us::UInt64
    t_off_us::UInt64
end

function GustStep(mean::AbstractWind, gust_v_ned::Vec3, t_on_s::Real, t_off_s::Real)
    t_on_us = _sec_to_us(t_on_s; allow_zero = true)
    t_off_us = _sec_to_us(t_off_s; allow_zero = true)
    t_off_us >= t_on_us ||
        throw(ArgumentError("GustStep requires t_off >= t_on (got $t_on_s..$t_off_s)"))
    return GustStep(mean, gust_v_ned, t_on_us, t_off_us)
end

function wind_velocity(w::GustStep, pos_ned::Vec3, t_us::UInt64)
    v = wind_velocity(w.mean, pos_ned, t_us)
    # Half-open interval avoids an extra "end tick" and makes scheduling behavior explicit.
    return (t_us >= w.t_on_us && t_us < w.t_off_us) ? (v + w.gust_v_ned) : v
end

function wind_velocity(w::GustStep, pos_ned::Vec3, t::Float64)
    return wind_velocity(w, pos_ned, _round_us(t))
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
    step_until_us::UInt64 = 0
    last_dt::Float64 = NaN
    last_tau_s::Float64 = NaN
    phi::Float64 = 0.0
    scale::Float64 = 0.0
end

function wind_velocity(w::OUWind, ::Vec3, t_us::UInt64)
    # Half-open end time to make duration unambiguous at tick boundaries.
    gust = (t_us < w.step_until_us) ? w.step_gust : vec3(0.0, 0.0, 0.0)
    return w.mean + w.v_gust + gust
end

function wind_velocity(w::OUWind, pos_ned::Vec3, t::Float64)
    return wind_velocity(w, pos_ned, _round_us(t))
end

@inline function _ensure_ou_coeffs!(w::OUWind, dt::Float64)
    if dt != w.last_dt || w.τ_s != w.last_tau_s
        if dt <= 0.0
            w.phi = 0.0
            w.scale = 0.0
        else
            τ = max(1e-6, w.τ_s)
            w.phi = exp(-dt / τ)
            w.scale = sqrt(max(0.0, 1.0 - w.phi^2))
        end
        w.last_dt = dt
        w.last_tau_s = w.τ_s
    end
end

function step_wind!(w::OUWind, ::Vec3, ::Float64, dt::Float64, rng::AbstractRNG)
    # Exact discrete-time OU update (dt-invariant stationary variance).
    #
    # For each axis:
    #   v[k+1] = ϕ v[k] + σ * sqrt(1-ϕ^2) * ξ,   ξ ~ N(0,1)
    # where ϕ = exp(-dt/τ).
    if dt <= 0.0
        _ensure_ou_coeffs!(w, dt)
        return nothing
    end

    _ensure_ou_coeffs!(w, dt)

    # Sample independent N(0,1) for each axis.
    w.v_gust =
        w.phi * w.v_gust + vec3(
            w.σ[1] * w.scale * randn(rng),
            w.σ[2] * w.scale * randn(rng),
            w.σ[3] * w.scale * randn(rng),
        )
    return nothing
end

"""Set the mean wind (mutable wind models only)."""
function set_mean_wind!(w::OUWind, v_ned::Vec3)
    w.mean = v_ned
    return nothing
end
set_mean_wind!(w::SampledWind, v_ned::Vec3) = set_mean_wind!(w.inner, v_ned)
set_mean_wind!(::AbstractWind, ::Vec3) =
    throw(ArgumentError("set_mean_wind! not supported for this wind type"))

"""Add a deterministic step gust for `duration_s` starting at the current time.

This is designed for scenario/event injection (e.g. a manual gust).
"""
function add_step_gust!(w::OUWind, dv_ned::Vec3, t_now_us::UInt64, duration_us::UInt64)
    w.step_gust = dv_ned
    w.step_until_us = t_now_us + duration_us
    return nothing
end

function add_step_gust!(w::OUWind, dv_ned::Vec3, t_now_us::UInt64, duration_s::Real)
    duration_us = _sec_to_us(max(0.0, Float64(duration_s)); allow_zero = true)
    return add_step_gust!(w, dv_ned, t_now_us, duration_us)
end

function add_step_gust!(w::OUWind, dv_ned::Vec3, t_now_s::Float64, duration_s::Real)
    # Float64 overload is for convenience only; internally we evaluate on integer microseconds.
    return add_step_gust!(w, dv_ned, _round_us(t_now_s), duration_s)
end

add_step_gust!(w::SampledWind, dv_ned::Vec3, t_now_us::UInt64, duration_us::UInt64) =
    add_step_gust!(w.inner, dv_ned, t_now_us, duration_us)
add_step_gust!(w::SampledWind, dv_ned::Vec3, t_now_us::UInt64, duration_s::Real) =
    add_step_gust!(w.inner, dv_ned, t_now_us, duration_s)
add_step_gust!(w::SampledWind, dv_ned::Vec3, t_now_s::Float64, duration_s::Real) =
    add_step_gust!(w.inner, dv_ned, t_now_s, duration_s)

add_step_gust!(::AbstractWind, ::Vec3, ::UInt64, ::UInt64) =
    throw(ArgumentError("add_step_gust! not supported for this wind type"))
add_step_gust!(::AbstractWind, ::Vec3, ::UInt64, ::Real) =
    throw(ArgumentError("add_step_gust! not supported for this wind type"))
add_step_gust!(::AbstractWind, ::Vec3, ::Float64, ::Real) =
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

`origin` defines the local NED reference in lat/lon/alt (MSL). This keeps the atmosphere
density and PX4 global coordinates consistent.
"""
struct EnvironmentModel{A<:AbstractAtmosphere,W<:AbstractWind,G<:AbstractGravity}
    atmosphere::A
    wind::W
    gravity::G
    origin::WorldOrigin
end

EnvironmentModel(;
    atmosphere::AbstractAtmosphere = ISA1976(),
    wind::AbstractWind = NoWind(),
    gravity::AbstractGravity = UniformGravity(9.80665),
    origin::WorldOrigin = WorldOrigin(),
    origin_alt_msl_m::Union{Nothing,Float64} = nothing,
) = begin
    if origin_alt_msl_m !== nothing
        if origin != WorldOrigin()
            @warn "origin_alt_msl_m ignored because origin is explicitly set" origin_alt_msl_m=origin_alt_msl_m origin=origin
        else
            origin = WorldOrigin(
                lat_deg = origin.lat_deg,
                lon_deg = origin.lon_deg,
                alt_msl_m = origin_alt_msl_m,
            )
        end
    end
    EnvironmentModel(atmosphere, wind, gravity, origin)
end

end # module Environment
