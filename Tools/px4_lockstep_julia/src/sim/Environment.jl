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

export AbstractAtmosphere, ISA1976,
       air_temperature, air_pressure, air_density,
       AbstractWind, NoWind, ConstantWind, GustStep, wind_velocity,
       AbstractGravity, UniformGravity, SphericalGravity, gravity_accel,
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

"""Constant wind velocity in NED (m/s)."""
struct ConstantWind <: AbstractWind
    v_ned::Vec3
end
wind_velocity(w::ConstantWind, ::Vec3, ::Float64) = w.v_ned

"""A simple step gust.

Adds `gust_v_ned` between `[t_on, t_off]`.

This is intentionally simple and deterministic; you can replace it with Dryden/von Karman
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

This model expects `r` to be measured from Earth's center. For NED local frames, it's
usually adequate to use `UniformGravity` unless you're doing high-altitude/long-range.
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

"""A composable environment model."""
struct EnvironmentModel{A<:AbstractAtmosphere, W<:AbstractWind, G<:AbstractGravity}
    atmosphere::A
    wind::W
    gravity::G
end

EnvironmentModel(; atmosphere::AbstractAtmosphere=ISA1976(),
                  wind::AbstractWind=NoWind(),
                  gravity::AbstractGravity=UniformGravity(9.80665)) =
    EnvironmentModel(atmosphere, wind, gravity)

end # module Environment
