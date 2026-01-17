"""Sources.Estimator

Estimator/noise injection sources publish `bus.est`.

The engine treats `bus.est` as the state consumed by the autopilot.
This enables:
- truth-as-estimate (no estimator)
- estimator injection with noise/bias/delay (deterministic, RNG-sourced)
- full estimator replay (record `:est` on `timeline.ap` and replay without RNG)
"""

using Random: AbstractRNG

using ..Estimators: AbstractEstimator, estimate!
using ..Faults: SENSOR_FAULT_EST_FREEZE

"""Base type for estimator sources."""
abstract type AbstractEstimatorSource <: AbstractSource end

"""No-op estimator source.

Publishes truth-as-estimate into the bus so the autopilot always consumes `bus.est`.
"""
struct NullEstimatorSource <: AbstractEstimatorSource end

function update!(::NullEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    rb = _rb_state(plant_state)
    bus.est = EstimatedState(
        pos_ned = rb.pos_ned,
        vel_ned = rb.vel_ned,
        q_bn = rb.q_bn,
        ω_body = rb.ω_body,
    )
    return nothing
end

"""Replay estimator source.

Publishes `bus.est` at autopilot axis times.

Interpolation: estimator output is treated as ZOH between ticks.
"""
struct ReplayEstimatorSource{T<:ZOHTrace{EstimatedState}} <: AbstractEstimatorSource
    est_trace::T
end

function update!(src::ReplayEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.est = sample(src.est_trace, t_us)
    return nothing
end

"""Live estimator/noise injection source.

Calls `Sim.Estimators.estimate!` at autopilot ticks.

Determinism rules
-----------------
- RNG MUST be dedicated to this estimator source.
- No RNG usage is allowed outside of `update!`.

Fault interaction
-----------------
If `SENSOR_FAULT_EST_FREEZE` is set in `bus.faults.sensor_fault_mask`, the estimator
output is held constant (no update).
"""
struct LiveEstimatorSource{E,R} <: AbstractEstimatorSource
    est::E
    rng::R
    dt_est_s::Float64

    function LiveEstimatorSource(
        est::E,
        rng::R,
        dt_est_s::Real,
    ) where {E<:AbstractEstimator,R}
        return new{E,R}(est, rng, Float64(dt_est_s))
    end
end

function update!(src::LiveEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    if (bus.faults.sensor_fault_mask & SENSOR_FAULT_EST_FREEZE) != 0
        return nothing
    end
    rb = _rb_state(plant_state)
    t_s = Float64(t_us) * 1e-6
    bus.est = estimate!(src.est, src.rng, t_s, rb, src.dt_est_s)
    return nothing
end

export AbstractEstimatorSource,
    NullEstimatorSource, ReplayEstimatorSource, LiveEstimatorSource
