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
using ..Types: Vec3
using ..Vehicles: ActuatorCommand
using ..Powertrain: BatteryStatus
using ..Faults: FaultState
using ..Autopilots: AutopilotCommand
using ..Estimators: EstimatedState
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

mutable struct StreamCursor{T}
    axis::TimeAxis
    idx::Int
    values::Vector{T}
    name::Symbol
end

function StreamCursor(axis::TimeAxis, ::Type{T}, name::Symbol) where {T}
    return StreamCursor{T}(axis, 1, Vector{T}(undef, length(axis.t_us)), name)
end

@inline function record!(cur::StreamCursor{T}, t_us::UInt64, value::T) where {T}
    idx = cur.idx
    idx <= length(cur.values) ||
        error("Recorder stream $(cur.name) overflow: idx=$(idx) len=$(length(cur.values))")
    axis_t = cur.axis.t_us[idx]
    axis_t == t_us || error(
        "Recorder stream $(cur.name) time mismatch: expected=$(axis_t) got=$(t_us)",
    )
    cur.values[idx] = value
    cur.idx = idx + 1
    return nothing
end

mutable struct FixedRecorder{PS} <: AbstractRecorder
    timeline::Timeline

    cmd::StreamCursor{ActuatorCommand}
    wind_ned::StreamCursor{Vec3}
    plant::StreamCursor{PS}
    battery::StreamCursor{BatteryStatus}

    faults::StreamCursor{FaultState}
    ap_cmd::StreamCursor{AutopilotCommand}
    landed::StreamCursor{Bool}
    wind_dist::StreamCursor{Vec3}

    faults_evt::Union{Nothing,StreamCursor{FaultState}}
    ap_cmd_evt::Union{Nothing,StreamCursor{AutopilotCommand}}
    landed_evt::Union{Nothing,StreamCursor{Bool}}
    wind_dist_evt::Union{Nothing,StreamCursor{Vec3}}

    est::Union{Nothing,StreamCursor{EstimatedState}}
end

function FixedRecorder(
    timeline::Timeline,
    plant0;
    record_faults_evt::Bool = true,
    record_estimator::Bool = false,
)
    cmd = StreamCursor(timeline.ap, ActuatorCommand, :cmd)
    wind_ned = StreamCursor(timeline.wind, Vec3, :wind_ned)
    plant = StreamCursor(timeline.log, typeof(plant0), :plant)
    battery = StreamCursor(timeline.log, BatteryStatus, :battery)

    faults = StreamCursor(timeline.scn, FaultState, :faults)
    ap_cmd = StreamCursor(timeline.scn, AutopilotCommand, :ap_cmd)
    landed = StreamCursor(timeline.scn, Bool, :landed)
    wind_dist = StreamCursor(timeline.scn, Vec3, :wind_dist)

    faults_evt = record_faults_evt ? StreamCursor(timeline.evt, FaultState, :faults_evt) : nothing
    ap_cmd_evt = record_faults_evt ? StreamCursor(timeline.evt, AutopilotCommand, :ap_cmd_evt) : nothing
    landed_evt = record_faults_evt ? StreamCursor(timeline.evt, Bool, :landed_evt) : nothing
    wind_dist_evt = record_faults_evt ? StreamCursor(timeline.evt, Vec3, :wind_dist_evt) : nothing

    est = record_estimator ? StreamCursor(timeline.ap, EstimatedState, :est) : nothing

    return FixedRecorder(
        timeline,
        cmd,
        wind_ned,
        plant,
        battery,
        faults,
        ap_cmd,
        landed,
        wind_dist,
        faults_evt,
        ap_cmd_evt,
        landed_evt,
        wind_dist_evt,
        est,
    )
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

@inline record!(rec::FixedRecorder, name::Symbol, t_us::UInt64, value) =
    record!(rec, Val(name), t_us, value)

@inline record!(::FixedRecorder, ::Val{name}, ::UInt64, ::Any) where {name} =
    error("Recorder has no stream named $(name)")

@inline record!(rec::FixedRecorder, ::Val{:cmd}, t_us::UInt64, value::ActuatorCommand) =
    record!(rec.cmd, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:wind_ned}, t_us::UInt64, value::Vec3) =
    record!(rec.wind_ned, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:plant}, t_us::UInt64, value) =
    record!(rec.plant, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:battery}, t_us::UInt64, value::BatteryStatus) =
    record!(rec.battery, t_us, value)

@inline record!(rec::FixedRecorder, ::Val{:faults}, t_us::UInt64, value::FaultState) =
    record!(rec.faults, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:ap_cmd}, t_us::UInt64, value::AutopilotCommand) =
    record!(rec.ap_cmd, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:landed}, t_us::UInt64, value::Bool) =
    record!(rec.landed, t_us, value)
@inline record!(rec::FixedRecorder, ::Val{:wind_dist}, t_us::UInt64, value::Vec3) =
    record!(rec.wind_dist, t_us, value)

@inline record!(rec::FixedRecorder, ::Val{:faults_evt}, t_us::UInt64, value::FaultState) =
    (rec.faults_evt === nothing ? nothing : record!(rec.faults_evt, t_us, value))
@inline record!(rec::FixedRecorder, ::Val{:ap_cmd_evt}, t_us::UInt64, value::AutopilotCommand) =
    (rec.ap_cmd_evt === nothing ? nothing : record!(rec.ap_cmd_evt, t_us, value))
@inline record!(rec::FixedRecorder, ::Val{:landed_evt}, t_us::UInt64, value::Bool) =
    (rec.landed_evt === nothing ? nothing : record!(rec.landed_evt, t_us, value))
@inline record!(rec::FixedRecorder, ::Val{:wind_dist_evt}, t_us::UInt64, value::Vec3) =
    (rec.wind_dist_evt === nothing ? nothing : record!(rec.wind_dist_evt, t_us, value))

@inline record!(rec::FixedRecorder, ::Val{:est}, t_us::UInt64, value::EstimatedState) =
    (rec.est === nothing ? nothing : record!(rec.est, t_us, value))

"""Finalize the recorder.

For InMemoryRecorder this is currently a no-op.
A future implementation may:
- validate schema
- freeze internal buffers
"""
function finalize!(::InMemoryRecorder)
    return nothing
end

function finalize!(::FixedRecorder)
    return nothing
end

function _flush_stream!(
    out::InMemoryRecorder,
    name::Symbol,
    cur::StreamCursor,
)
    cur.idx == length(cur.values) + 1 || error(
        "Recorder stream $name is incomplete: idx=$(cur.idx) len=$(length(cur.values))",
    )
    out.times[name] = cur.axis.t_us
    out.values[name] = cur.values
    return nothing
end

function to_inmemory(rec::FixedRecorder)
    out = InMemoryRecorder()
    _flush_stream!(out, :cmd, rec.cmd)
    _flush_stream!(out, :wind_ned, rec.wind_ned)
    _flush_stream!(out, :plant, rec.plant)
    _flush_stream!(out, :battery, rec.battery)
    _flush_stream!(out, :faults, rec.faults)
    _flush_stream!(out, :ap_cmd, rec.ap_cmd)
    _flush_stream!(out, :landed, rec.landed)
    _flush_stream!(out, :wind_dist, rec.wind_dist)
    if rec.faults_evt !== nothing
        _flush_stream!(out, :faults_evt, rec.faults_evt)
        _flush_stream!(out, :ap_cmd_evt, rec.ap_cmd_evt)
        _flush_stream!(out, :landed_evt, rec.landed_evt)
        _flush_stream!(out, :wind_dist_evt, rec.wind_dist_evt)
    end
    if rec.est !== nothing
        _flush_stream!(out, :est, rec.est)
    end
    return out
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

stream_times(rec::FixedRecorder, name::Symbol) = stream_times(to_inmemory(rec), name)
stream_values(rec::FixedRecorder, name::Symbol) = stream_values(to_inmemory(rec), name)

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
    wind_ned = samplehold_trace(rec, :wind_ned, timeline.wind)
    plant = sampled_trace(rec, :plant, timeline.log)
    battery = sampled_trace(rec, :battery, timeline.log)
    return (; cmd, wind_ned, plant, battery)
end

tier0_traces(rec::FixedRecorder, timeline::Timeline) = tier0_traces(to_inmemory(rec), timeline)

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

scenario_traces(rec::FixedRecorder, timeline::Timeline) =
    scenario_traces(to_inmemory(rec), timeline)

"""Build estimator output trace from an in-memory recorder."""
function estimator_traces(rec::InMemoryRecorder, timeline::Timeline)
    est = zoh_trace(rec, :est, timeline.ap)
    return (; est)
end

estimator_traces(rec::FixedRecorder, timeline::Timeline) =
    estimator_traces(to_inmemory(rec), timeline)


export AbstractRecorder, NullRecorder, InMemoryRecorder
export FixedRecorder, to_inmemory
export stream_times, stream_values
export zoh_trace, samplehold_trace, sampled_trace
export tier0_traces, scenario_traces, estimator_traces
