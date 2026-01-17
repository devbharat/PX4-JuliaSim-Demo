"""Recording.Recording

Recording containers and persistence helpers.

The goal is to make record/replay a **first-class workflow**, not a loose collection
of CSV dumps.

Design constraints
------------------
- Keep dependencies minimal (no HDF5 required for the initial pass).
- Round-trip deterministically (stable ordering for Dict-like payloads).

Persistence format
------------------
This first pass uses Julia `Serialization` (binary blob). It is fast and dependency-free,
but not intended as a long-lived archival format.

Once recordings become large (minutes+ at high rates), migrate to an HDF5-backed schema.
"""

using Serialization

using ..Runtime: BUS_SCHEMA_VERSION, Timeline

# Local helpers for validating loaded recordings.
using ..Runtime: TimeAxis

"""Tier-0 recording: sufficient to replay the plant deterministically.

This bundles:
- the `Timeline`
- the initial plant state
- an in-memory recorder containing sampled streams (cmd, wind, plant, ...)

Notes
-----
- Model parameters are NOT persisted (replay assumes current model).
- For cross-version compatibility, prefer a future HDF5 format.
"""
Base.@kwdef struct Tier0Recording{PS}
    bus_schema_version::Int = BUS_SCHEMA_VERSION
    timeline::Timeline
    plant0::PS
    recorder::InMemoryRecorder
    meta::Dict{Symbol,Any} = Dict{Symbol,Any}()
end

"""Serialization-friendly Tier-0 container with deterministic ordering."""
Base.@kwdef struct Tier0RecordingSerialized{PS}
    bus_schema_version::Int
    timeline::Timeline
    plant0::PS
    recorder_times::Vector{Pair{Symbol,Vector{UInt64}}}
    recorder_values::Vector{Pair{Symbol,Any}}
    meta::Vector{Pair{Symbol,Any}}
end

@inline function _sorted_pairs(d::Dict{Symbol,T}) where {T}
    keys_sorted = sort!(collect(keys(d)); by = String)
    out = Vector{Pair{Symbol,T}}(undef, length(keys_sorted))
    for (i, k) in enumerate(keys_sorted)
        out[i] = k => d[k]
    end
    return out
end

function _to_serialized(rec::Tier0Recording)
    rec_times = _sorted_pairs(rec.recorder.times)
    rec_values = _sorted_pairs(rec.recorder.values)
    meta = _sorted_pairs(rec.meta)
    return Tier0RecordingSerialized(
        bus_schema_version = rec.bus_schema_version,
        timeline = rec.timeline,
        plant0 = rec.plant0,
        recorder_times = rec_times,
        recorder_values = rec_values,
        meta = meta,
    )
end

function _from_serialized(rec::Tier0RecordingSerialized)
    recorder = InMemoryRecorder()
    for (name, t) in rec.recorder_times
        recorder.times[name] = t
    end
    for (name, v) in rec.recorder_values
        recorder.values[name] = v
    end
    meta = Dict{Symbol,Any}(rec.meta)
    return Tier0Recording(
        bus_schema_version = rec.bus_schema_version,
        timeline = rec.timeline,
        plant0 = rec.plant0,
        recorder = recorder,
        meta = meta,
    )
end

"""Save a recording to disk (binary serialization)."""
function save_recording(path::AbstractString, rec::Tier0Recording)
    open(path, "w") do io
        serialize(io, _to_serialized(rec))
    end
    return nothing
end

"""Load a recording from disk (binary serialization)."""
function load_recording(path::AbstractString)::Tier0Recording
    rec = open(path, "r") do io
        deserialize(io)
    end
    if rec isa Tier0RecordingSerialized
        rec = _from_serialized(rec)
    elseif !(rec isa Tier0Recording)
        error("Expected Tier0Recording, got $(typeof(rec))")
    end
    rec.bus_schema_version == BUS_SCHEMA_VERSION || error(
        "Recording schema mismatch: recording has $(rec.bus_schema_version), expected $(BUS_SCHEMA_VERSION)",
    )

    # Lightweight structural validation. Fail fast with a clear message if the
    # recording cannot be replayed by the current engine.
    validate_recording(rec)
    return rec
end

"""Validate that a Tier-0 recording is internally consistent.

This is intentionally *lightweight* and focuses on catching the most common
causes of confusing replay bugs:

- timeline axes not strictly increasing
- missing required streams
- stream time axes not matching timeline axes exactly

It does **not** attempt to validate physics/model parameters.

Throws an ErrorException on failure.
"""
function validate_recording(rec::Tier0Recording)
    # 1) Schema version
    rec.bus_schema_version == BUS_SCHEMA_VERSION || error(
        "Recording schema mismatch: recording has $(rec.bus_schema_version), expected $(BUS_SCHEMA_VERSION)",
    )

    # 2) Timeline monotonicity
    _validate_axis(rec.timeline.ap)
    _validate_axis(rec.timeline.wind)
    _validate_axis(rec.timeline.log)
    _validate_axis(rec.timeline.scn)
    _validate_axis(rec.timeline.phys)
    _validate_axis(rec.timeline.evt)

    # 3) Required Tier-0 streams
    # These are required for plant replay.
    # (If you want a "boundary-only" recording, add a separate format.)
    try
        tier0_traces(rec)
    catch e
        error(
            "Recording validation failed while building tier0_traces: $(sprint(showerror, e))",
        )
    end

    # 4) Optional scenario streams (validate if present)
    if haskey(rec.recorder.times, :faults_evt) || haskey(rec.recorder.times, :faults)
        try
            scenario_traces(rec)
        catch e
            error(
                "Recording validation failed while building scenario_traces: $(sprint(showerror, e))",
            )
        end
    end

    # 5) Optional estimator streams (validate if present)
    if haskey(rec.recorder.times, :est)
        try
            estimator_traces(rec)
        catch e
            error(
                "Recording validation failed while building estimator_traces: $(sprint(showerror, e))",
            )
        end
    end

    return nothing
end

@inline function _validate_axis(axis::TimeAxis)
    t = axis.t_us
    isempty(t) && error("Timeline axis $(axis.name) is empty")
    # Strictly increasing, since axes represent event boundaries.
    for i = 2:length(t)
        t[i] > t[i - 1] || error(
            "Timeline axis $(axis.name) is not strictly increasing at i=$i: $(t[i-1]) -> $(t[i])",
        )
    end
    return nothing
end

# Convenience trace builders

tier0_traces(rec::Tier0Recording) = tier0_traces(rec.recorder, rec.timeline)
scenario_traces(rec::Tier0Recording) = scenario_traces(rec.recorder, rec.timeline)
estimator_traces(rec::Tier0Recording) = estimator_traces(rec.recorder, rec.timeline)

export Tier0Recording, save_recording, load_recording
export tier0_traces, scenario_traces, estimator_traces
export validate_recording
