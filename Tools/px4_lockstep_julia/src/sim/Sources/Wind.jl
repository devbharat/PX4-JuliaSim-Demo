"""Sources.Wind

Wind sources publish wind velocity samples into the runtime bus.

Two modes:
- **Live**: step a deterministic wind model (with dedicated RNG) at `timeline.wind`.
- **Replay**: sample a recorded wind trace and publish it deterministically.

The wind published on the bus is treated as **sample-and-hold** between wind ticks.
"""

using Random: AbstractRNG

using ..Environment: AbstractWind, step_wind!, sample_wind!, wind_velocity

"""Base type for wind sources."""
abstract type AbstractWindSource <: AbstractSource end

"""Live wind source.

Advances the deterministic wind model once per wind tick and publishes the sampled
wind velocity into `bus.wind_ned`.

Determinism rules
-----------------
- The RNG MUST be dedicated to this wind source.
- No RNG usage is allowed outside of `update!`.
"""
mutable struct LiveWindSource{W,R} <: AbstractWindSource
    wind::W
    rng::R
    dt_wind_s::Float64

    function LiveWindSource(wind::W, rng::R, dt_wind_s::Real) where {W<:AbstractWind,R}
        return new{W,R}(wind, rng, Float64(dt_wind_s))
    end
end

function update!(src::LiveWindSource, bus::SimBus, plant_state, t_us::UInt64)
    rb = _rb_state(plant_state)
    t_s = Float64(t_us) * 1e-6
    pos = rb.pos_ned

    step_wind!(src.wind, pos, t_s, src.dt_wind_s, src.rng)
    sample_wind!(src.wind, pos, t_s)
    bus.wind_ned = wind_velocity(src.wind, pos, t_s)
    return nothing
end

"""Replay wind source.

Samples a sample-and-hold wind trace and publishes into `bus.wind_ned`.
"""
struct ReplayWindSource{T<:SampleHoldTrace{Vec3}} <: AbstractWindSource
    wind_trace::T
end

function update!(src::ReplayWindSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.wind_ned = sample(src.wind_trace, t_us)
    return nothing
end

export AbstractWindSource, LiveWindSource, ReplayWindSource
