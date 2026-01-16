"""Recording containers and persistence helpers.

The goal is to make record/replay a **first-class workflow**, not just a loose
collection of traces.

Design constraints
------------------
* Keep dependencies minimal (no HDF5 required for the initial shell).
* Make it easy to round-trip a recording through a file so it can be used for:
  - integrator sweeps
  - regression tests
  - offline analysis

Persistence format
------------------
We currently use Julia's built-in `Serialization` for a simple binary blob.
This is:
* fast
* dependency-free
* not intended as a long-lived stable format

Once recordings become large (minutes+ at high rates), we expect to add an
HDF5-backed recorder/writer (see `docs/record_replay_todo.md`).
"""

using Serialization

"""Tier-0 recording: sufficient to replay the plant deterministically.

This bundles:
* the timeline
* the initial plant state
* an in-memory recorder containing sampled streams (cmd, wind, plant, ...)

Notes
-----
* The *model parameters* are not persisted here. Replay assumes "replay against
  the current model".
* For long-term archival or cross-version compatibility, prefer a future HDF5
  format.
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
    return rec
end

"""Convenience: build Tier-0 traces from a `Tier0Recording`."""
tier0_traces(rec::Tier0Recording) = tier0_traces(rec.recorder, rec.timeline)

"""Convenience: build scenario output traces from a `Tier0Recording`."""
scenario_traces(rec::Tier0Recording) = scenario_traces(rec.recorder, rec.timeline)

"""Convenience: build estimator output traces from a `Tier0Recording`."""
estimator_traces(rec::Tier0Recording) = estimator_traces(rec.recorder, rec.timeline)
