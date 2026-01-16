"""Recorders for Option A record/replay.

A recorder captures a set of **streams** aligned to time axes.

This file defines:
- `AbstractRecorder` interface
- `NullRecorder` (no-op)
- `InMemoryRecorder` (small deterministic capture suitable for tests)

HDF5 support is planned but intentionally not implemented yet to keep dependencies
minimal. See `docs/record_replay_todo.md`.
"""

# NOTE: We are in `PX4Lockstep.Sim.RecordReplay`.

abstract type AbstractRecorder end

"""No-op recorder."""
struct NullRecorder <: AbstractRecorder end

@inline record!(::NullRecorder, ::Symbol, ::UInt64, ::Any) = nothing
@inline finalize!(::NullRecorder) = nothing

"""In-memory deterministic recorder.

This recorder stores streams as simple vectors in memory.
It is intended for:
- unit tests
- small debug runs

It is **not** intended for long missions.

Design
------
We store:
- `times[name]::Vector{UInt64}`
- `values[name]::Vector{T}`

and later convert them into `Trace` objects.

TODO
----
- Enforce that recorded times match the expected timeline axes.
- Add helpers that directly build `ZOHTrace/SampledTrace/SampleHoldTrace`.
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

"""Build the standard Tier-0 traces from an in-memory recorder.

Expected streams
----------------
- `:cmd`      on `timeline.ap`
- `:wind_ned` on `timeline.wind`
- `:plant`    on `timeline.log`
- `:battery`  on `timeline.log`

Returns
-------
NamedTuple `(cmd, wind_ned, plant, battery)` of trace objects.
"""
function tier0_traces(rec::InMemoryRecorder, timeline::Timeline)
    cmd = zoh_trace(rec, :cmd, timeline.ap)
    wind_ned = samplehold_trace(rec, :wind_ned, timeline.wind)
    plant = sampled_trace(rec, :plant, timeline.log)
    battery = sampled_trace(rec, :battery, timeline.log)
    return (; cmd, wind_ned, plant, battery)
end

"""Build scenario output traces from an in-memory recorder.

Scenario outputs are recorded as **bus streams**.

Preferred (robust) streams
--------------------------
If present, we prefer the `*_evt` streams aligned to `timeline.evt`:

* `:ap_cmd_evt`  on `timeline.evt`
* `:landed_evt`  on `timeline.evt`
* `:faults_evt`  on `timeline.evt`

These capture dynamic scenario outputs (e.g. `When(...)`-driven fault changes) that
may change at arbitrary event boundaries.

Fallback streams
----------------
For older recordings (or tiny static scenarios), we fall back to the small
scenario-axis streams:

* `:ap_cmd`  on `timeline.scn`
* `:landed`  on `timeline.scn`
* `:faults`  on `timeline.scn`

Returns
-------
NamedTuple `(ap_cmd, landed, faults)` containing `ZOHTrace`s.
"""
function scenario_traces(rec::InMemoryRecorder, timeline::Timeline)
    if haskey(rec.times, :faults_evt)
        ap_cmd = zoh_trace(rec, :ap_cmd_evt, timeline.evt)
        landed = zoh_trace(rec, :landed_evt, timeline.evt)
        faults = zoh_trace(rec, :faults_evt, timeline.evt)
        return (; ap_cmd, landed, faults)
    end

    ap_cmd = zoh_trace(rec, :ap_cmd, timeline.scn)
    landed = zoh_trace(rec, :landed, timeline.scn)
    faults = zoh_trace(rec, :faults, timeline.scn)
    return (; ap_cmd, landed, faults)
end

"""Build estimator output trace from an in-memory recorder.

Expected streams
----------------
* `:est` on `timeline.ap`

Returns
-------
NamedTuple `(est,)` containing a `SampledTrace`.

Notes
-----
This is optional and is typically recorded only in Tier-1+.
"""
function estimator_traces(rec::InMemoryRecorder, timeline::Timeline)
    est = sampled_trace(rec, :est, timeline.ap)
    return (; est)
end

"""Planned: write recorder contents to a persistent file.

TODO: implement HDF5 backend.
"""
function write_recording(::AbstractRecorder, ::AbstractString)
    error("TODO: write_recording is not implemented yet (planned HDF5 backend)")
end
