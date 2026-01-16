"""Trace (stream) abstractions for record/replay.

A **trace** is a time-indexed stream of values aligned to a `TimeAxis`.

Trace types
-----------
- `SampledTrace`: values are defined only at axis points; sampling requires exact match.
- `ZOHTrace`: zero-order hold; value at time `t` is last sample `<= t`.
- `SampleHoldTrace`: same as ZOH but used to communicate semantics (wind samples).

All sampling semantics are **deterministic** and defined in integer microseconds.
"""

# NOTE: We are in `PX4Lockstep.Sim.RecordReplay`.

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

Same evaluation semantics as ZOH, but used for wind/disturbance realizations.
"""
struct SampleHoldTrace{T} <: AbstractTrace{T}
    axis::TimeAxis
    data::Vector{T}

    function SampleHoldTrace{T}(axis::TimeAxis, data::Vector{T}) where {T}
        _check_trace_lengths(axis, data)
        return new{T}(axis, data)
    end
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

function sample(tr::ZOHTrace{T}, t_us::UInt64)::T where {T}
    i = _last_leq_index(tr.axis.t_us, t_us)
    i == 0 &&
        error("ZOHTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)")
    return tr.data[i]
end

function sample(tr::SampleHoldTrace{T}, t_us::UInt64)::T where {T}
    i = _last_leq_index(tr.axis.t_us, t_us)
    i == 0 && error(
        "SampleHoldTrace requested before first sample: t_us=$t_us axis=$(tr.axis.name)",
    )
    return tr.data[i]
end
