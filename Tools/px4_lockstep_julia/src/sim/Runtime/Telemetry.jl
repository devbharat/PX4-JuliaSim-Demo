"""Runtime.Telemetry

Optional boundary-time telemetry hooks.

Why this exists
---------------
It is common to want additional "derived" quantities or side-channel reporting
without perturbing simulation semantics (and therefore determinism).

Instead of letting ad-hoc code sprinkle callbacks into the engine, we model
telemetry as an **explicit boundary stage** (`:telemetry`) in the canonical stage
order. The stage is always present, but is a no-op unless a telemetry sink is
provided.

Determinism contract
--------------------
Telemetry hooks **must not**:
- mutate the plant state
- mutate the bus
- access randomness
- depend on wall-clock time

They are intended for:
- metrics
- debug printing
- mirroring state into external buffers

If you need a hook that *does* affect the simulation, it should be expressed as a
bus-level signal published by a Source (scenario/wind/estimator/autopilot), not as
telemetry.
"""

abstract type AbstractTelemetrySink end

"""No-op telemetry sink (default)."""
struct NullTelemetry <: AbstractTelemetrySink end

@inline telemetry!(::NullTelemetry, _bus, _plant, _t_us::UInt64) = nothing

"""Composite telemetry sink that forwards to multiple sinks in a fixed order."""
struct CompositeTelemetry{T<:Tuple} <: AbstractTelemetrySink
    sinks::T
end

@inline function telemetry!(t::CompositeTelemetry, bus, plant, t_us::UInt64)
    for s in t.sinks
        telemetry!(s, bus, plant, t_us)
    end
    return nothing
end

"""Allow a plain function to act as a telemetry sink.

`f(bus, plant, t_us)` will be called at telemetry boundaries.
"""
@inline telemetry!(f::Function, bus, plant, t_us::UInt64) = f(bus, plant, t_us)

export AbstractTelemetrySink, NullTelemetry, CompositeTelemetry, telemetry!
