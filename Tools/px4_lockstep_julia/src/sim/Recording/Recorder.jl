"""Recording.Recorder

Recorders for Option A record/replay.

A recorder captures a set of **streams** aligned to time axes.

This file defines:
- `AbstractRecorder` interface
- `NullRecorder` (no-op)
- `InMemoryRecorder` (deterministic capture suitable for tests and small runs)

Persistence backends
--------------------
The dependency-light first pass uses an in-memory recorder + Julia Serialization.
For large logs/long missions, add an HDF5-backed recorder/writer (planned).
"""

using ..Runtime: TimeAxis, Timeline
import ..Runtime: record!, finalize!

abstract type AbstractRecorder end

"""No-op recorder."""
struct NullRecorder <: AbstractRecorder end

@inline record!(::NullRecorder, ::Symbol, ::UInt64, ::Any) = nothing
@inline finalize!(::NullRecorder) = nothing

"""In-memory deterministic recorder.

This recorder stores streams as vectors in memory and is intended for:
- unit tests
- short debug runs

It is **not** intended for long missions.

Design
------
The recorder stores:
- `times[name]::Vector{UInt64}`
- `values[name]::Vector{T}`

and later convert them into `Trace` objects.

TODO
----
- Enforce that recorded times match the expected timeline axes.
- Add an HDF5 backend for large runs.
"""
mutable struct InMemoryRecorder <: AbstractRecorder
    times::Dict{Symbol,Vector{UInt64}}
    values::Dict{Symbol,Any}
end

function InMemoryRecorder()
    return InMemoryRecorder(Dict{Symbol,Vector{UInt64}}(), Dict{Symbol,Any}())
end

function record!(rec::InMemoryRecorder, name::Symbol, t_us::UInt64, value)
    ts = get!(rec.times, name) do
        UInt64[]
    end
    vs = get!(rec.values, name) do
        Vector{typeof(value)}()
    end

    # Determinism guard: times must be non-decreasing. Exact axis membership is validated later.
    if !isempty(ts) && t_us < ts[end]
        error("Recorder stream $name saw non-monotone time: last=$(ts[end]) new=$t_us")
    end

    push!(ts, t_us)
    push!(vs, value)
    return nothing
end

"""Finalize the recorder.

For InMemoryRecorder this is currently a no-op.
A future implementation may:
- validate schema
- freeze internal buffers
"""
function finalize!(::InMemoryRecorder)
    return nothing
end

# ------------------------------------------------------------
# Trace builders (InMemoryRecorder -> Trace)
# ------------------------------------------------------------

"""Return the recorded times for `name`.

Throws if the stream is missing.
"""
function stream_times(rec::InMemoryRecorder, name::Symbol)::Vector{UInt64}
    haskey(rec.times, name) || error("Recorder has no stream named $name")
    return rec.times[name]
end

"""Return the recorded values for `name`.

Throws if the stream is missing.
"""
function stream_values(rec::InMemoryRecorder, name::Symbol)
    haskey(rec.values, name) || error("Recorder has no stream named $name")
    return rec.values[name]
end

@inline function _require_axis_match!(name::Symbol, axis::TimeAxis, t_rec::Vector{UInt64})
    axis.t_us == t_rec || error(
        "Recorder stream $name times do not match axis $(axis.name):\n" *
        "  axis[1:3]=$(axis.t_us[1:min(end,3)]) ... axis[end]=$(axis.t_us[end])\n" *
        "  rec [1:3]=$(t_rec[1:min(end,3)]) ... rec [end]=$(t_rec[end])",
    )
    return nothing
end

"""Build a `ZOHTrace` from a recorded stream aligned exactly to `axis`."""
function zoh_trace(rec::InMemoryRecorder, name::Symbol, axis::TimeAxis)
    t = stream_times(rec, name)
    _require_axis_match!(name, axis, t)
    v = stream_values(rec, name)
    return ZOHTrace(axis, v)
end

"""Build a `SampleHoldTrace` from a recorded stream aligned exactly to `axis`."""
function samplehold_trace(rec::InMemoryRecorder, name::Symbol, axis::TimeAxis)
    t = stream_times(rec, name)
    _require_axis_match!(name, axis, t)
    v = stream_values(rec, name)
    return SampleHoldTrace(axis, v)
end

"""Build a `SampledTrace` from a recorded stream aligned exactly to `axis`."""
function sampled_trace(rec::InMemoryRecorder, name::Symbol, axis::TimeAxis)
    t = stream_times(rec, name)
    _require_axis_match!(name, axis, t)
    v = stream_values(rec, name)
    return SampledTrace(axis, v)
end

"""Build the standard Tier-0 traces from an in-memory recorder."""
function tier0_traces(rec::InMemoryRecorder, timeline::Timeline)
    cmd = zoh_trace(rec, :cmd, timeline.ap)
    wind_base_ned = samplehold_trace(rec, :wind_base_ned, timeline.wind)
    plant = sampled_trace(rec, :plant, timeline.log)
    battery = sampled_trace(rec, :battery, timeline.log)
    batteries = sampled_trace(rec, :batteries, timeline.log)
    return (; cmd, wind_base_ned, plant, battery, batteries)
end

"""Build scenario output traces from an in-memory recorder."""
function scenario_traces(rec::InMemoryRecorder, timeline::Timeline)
    if haskey(rec.times, :faults_evt)
        ap_cmd = zoh_trace(rec, :ap_cmd_evt, timeline.evt)
        landed = zoh_trace(rec, :landed_evt, timeline.evt)
        faults = zoh_trace(rec, :faults_evt, timeline.evt)
        if haskey(rec.times, :wind_dist_evt)
            wind_dist = zoh_trace(rec, :wind_dist_evt, timeline.evt)
            return (; ap_cmd, landed, faults, wind_dist)
        end
        return (; ap_cmd, landed, faults)
    end

    ap_cmd = zoh_trace(rec, :ap_cmd, timeline.scn)
    landed = zoh_trace(rec, :landed, timeline.scn)
    faults = zoh_trace(rec, :faults, timeline.scn)
    if haskey(rec.times, :wind_dist)
        wind_dist = zoh_trace(rec, :wind_dist, timeline.scn)
        return (; ap_cmd, landed, faults, wind_dist)
    end
    return (; ap_cmd, landed, faults)
end

"""Build estimator output trace from an in-memory recorder."""
function estimator_traces(rec::InMemoryRecorder, timeline::Timeline)
    est = zoh_trace(rec, :est, timeline.ap)
    return (; est)
end


export AbstractRecorder, NullRecorder, InMemoryRecorder
export stream_times, stream_values
export zoh_trace, samplehold_trace, sampled_trace
export tier0_traces, scenario_traces, estimator_traces
