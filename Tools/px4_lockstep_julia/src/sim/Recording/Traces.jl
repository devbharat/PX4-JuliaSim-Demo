"""Recording.Traces

Trace (stream) abstractions for record/replay.

A **trace** is a time-indexed stream of values aligned to a `Runtime.TimeAxis`.

Trace types
-----------
- `SampledTrace`: values are defined only at axis points; sampling requires exact match.
- `ZOHTrace`: zero-order hold; value at time `t` is last sample `<= t`.
- `SampleHoldTrace`: semantic alias of ZOH (used for disturbances like wind).

All sampling semantics are deterministic and defined in integer microseconds.
"""

using ..Runtime: TimeAxis

abstract type AbstractTrace{T} end

"""Trace that is only defined exactly on its axis."""
struct SampledTrace{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}

    function SampledTrace{T}(axis::TimeAxis, data::Vector{T}) where {T}
        _check_trace_lengths(axis, data)
        return new{T}(axis, data)
    end
end

"""Zero-order hold trace.

Sampling returns the most recent value with timestamp `<= t_us`.
"""
struct ZOHTrace{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}

    function ZOHTrace{T}(axis::TimeAxis, data::Vector{T}) where {T}
        _check_trace_lengths(axis, data)
        return new{T}(axis, data)
    end
end

"""Sample-and-hold trace.

Same evaluation semantics as ZOH, but used to communicate semantics.
"""
struct SampleHoldTrace{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}

    function SampleHoldTrace{T}(axis::TimeAxis, data::Vector{T}) where {T}
        _check_trace_lengths(axis, data)
        return new{T}(axis, data)
    end
end

"""Cursor-based sampled trace for monotonic replay."""
mutable struct SampledCursor{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}
    idx::Int
end

"""Cursor-based ZOH trace for monotonic replay."""
mutable struct ZOHCursor{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}
    idx::Int
end

"""Cursor-based sample-and-hold trace for monotonic replay."""
mutable struct SampleHoldCursor{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}
    idx::Int
end

function _check_trace_lengths(axis::TimeAxis, data::Vector)
    length(axis.t_us) == length(data) || throw(
        ArgumentError(
            "trace length mismatch: axis has $(length(axis.t_us)), data has $(length(data))",
        ),
    )
    return nothing
end

SampledTrace(axis::TimeAxis, data::Vector{T}) where {T} = SampledTrace{T}(axis, data)
ZOHTrace(axis::TimeAxis, data::Vector{T}) where {T} = ZOHTrace{T}(axis, data)
SampleHoldTrace(axis::TimeAxis, data::Vector{T}) where {T} = SampleHoldTrace{T}(axis, data)

SampledCursor(tr::SampledTrace{T}) where {T} = SampledCursor{T}(tr.axis, tr.data, 1)
ZOHCursor(tr::ZOHTrace{T}) where {T} = ZOHCursor{T}(tr.axis, tr.data, 1)
SampleHoldCursor(tr::SampleHoldTrace{T}) where {T} =
    SampleHoldCursor{T}(tr.axis, tr.data, 1)

# ----------------
# Sampling helpers
# ----------------

"""Find the index of the last axis time `<= t_us`.

Returns 0 if `t_us` is earlier than the first sample.
"""
@inline function _last_leq_index(t_axis::Vector{UInt64}, t_us::UInt64)::Int
    n = length(t_axis)
    n == 0 && return 0

    if t_us < t_axis[1]
        return 0
    elseif t_us >= t_axis[end]
        return n
    end

    lo = 1
    hi = n
    while lo <= hi
        mid = (lo + hi) >>> 1
        v = t_axis[mid]
        if v == t_us
            return mid
        elseif v < t_us
            lo = mid + 1
        else
            hi = mid - 1
        end
    end
    # `hi` ends at last <=
    return hi
end

"""Find index of an exact match on the axis.

Returns 0 if not found.
"""
@inline function _exact_index(t_axis::Vector{UInt64}, t_us::UInt64)::Int
    lo = 1
    hi = length(t_axis)
    while lo <= hi
        mid = (lo + hi) >>> 1
        v = t_axis[mid]
        if v == t_us
            return mid
        elseif v < t_us
            lo = mid + 1
        else
            hi = mid - 1
        end
    end
    return 0
end

"""Sample a trace at `t_us`.

This is the primary API used by replay sources.
"""
function sample(tr::SampledTrace{T}, t_us::UInt64)::T where {T}
    i = _exact_index(tr.axis.t_us, t_us)
    i == 0 && error("SampledTrace is undefined at t_us=$t_us (axis=$(tr.axis.name))")
    return tr.data[i]
end

function sample(tr::SampledCursor{T}, t_us::UInt64)::T where {T}
    axis_t = tr.axis.t_us
    n = length(axis_t)
    if n > 0 && t_us <= axis_t[1] && tr.idx != 1
        tr.idx = 1
    end
    idx = tr.idx
    if idx <= n && axis_t[idx] == t_us
        tr.idx = idx + 1
        return tr.data[idx]
    end
    i = _exact_index(axis_t, t_us)
    i == 0 && error("SampledTrace is undefined at t_us=$t_us (axis=$(tr.axis.name))")
    tr.idx = i + 1
    return tr.data[i]
end

function sample(tr::ZOHTrace{T}, t_us::UInt64)::T where {T}
    i = _last_leq_index(tr.axis.t_us, t_us)
    i == 0 &&
        error("ZOHTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)")
    return tr.data[i]
end

function sample(tr::ZOHCursor{T}, t_us::UInt64)::T where {T}
    axis_t = tr.axis.t_us
    n = length(axis_t)
    n == 0 && error("ZOHTrace has no samples (axis=$(tr.axis.name))")
    if t_us <= axis_t[1] && tr.idx != 1
        tr.idx = 1
    end
    if t_us < axis_t[1]
        error("ZOHTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)")
    end
    idx = tr.idx
    if idx < 1
        idx = 1
    elseif idx > n
        idx = n
    end
    while idx < n && axis_t[idx + 1] <= t_us
        idx += 1
    end
    tr.idx = idx
    return tr.data[idx]
end

function sample(tr::SampleHoldTrace{T}, t_us::UInt64)::T where {T}
    i = _last_leq_index(tr.axis.t_us, t_us)
    i == 0 && error(
        "SampleHoldTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)",
    )
    return tr.data[i]
end

function sample(tr::SampleHoldCursor{T}, t_us::UInt64)::T where {T}
    axis_t = tr.axis.t_us
    n = length(axis_t)
    n == 0 && error("SampleHoldTrace has no samples (axis=$(tr.axis.name))")
    if t_us <= axis_t[1] && tr.idx != 1
        tr.idx = 1
    end
    if t_us < axis_t[1]
        error("SampleHoldTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)")
    end
    idx = tr.idx
    if idx < 1
        idx = 1
    elseif idx > n
        idx = n
    end
    while idx < n && axis_t[idx + 1] <= t_us
        idx += 1
    end
    tr.idx = idx
    return tr.data[idx]
end

export AbstractTrace, SampledTrace, ZOHTrace, SampleHoldTrace
export SampledCursor, ZOHCursor, SampleHoldCursor
export sample
