"""Time axes and hybrid event timeline.

This module centralizes the *authoritative* timebase used by record/replay:
**integer microseconds**.

Key ideas
---------
- All schedules are represented as vectors of `UInt64 time_us`.
- The simulation advances between times in `T_evt_us`, the union of all component axes.
- Scenario `AtTime` events are first-class axis points (hybrid-system correctness).

This file intentionally avoids referencing any specific PX4 or plant logic.
It is meant to be reusable for:
- full simulation record/replay
- component-only replay harnesses
- offline trace comparison tools
"""

# NOTE: We are in `PX4Lockstep.Sim.RecordReplay`.

"""Convert a Float64 seconds interval to integer microseconds.

The conversion must be exact for determinism:
`dt_s` must be representable as an integer number of microseconds.

This is deliberately stricter than "close enough". If you want a non-integer
microsecond timebase, this record/replay system is the wrong tool.
"""
@inline function dt_to_us(dt_s::Float64)::UInt64
    dt_s > 0 || throw(ArgumentError("dt_s must be > 0"))
    us = round(Int64, dt_s * 1e6)
    # Require exact microsecond representability.
    abs(dt_s - (Float64(us) * 1e-6)) <= 1e-15 || throw(
        ArgumentError(
            "dt_s=$dt_s is not an integer multiple of 1 µs (got us=$us -> $(Float64(us)*1e-6))",
        ),
    )
    us >= 1 || throw(ArgumentError("dt_s too small; must be >= 1 µs"))
    return UInt64(us)
end

"""A named time axis sampled at integer microseconds."""
struct TimeAxis
    name::Symbol
    t_us::Vector{UInt64}
end

"""Hybrid simulation timeline.

Fields
------
- `t0_us`, `t_end_us`: inclusive start, inclusive end.
- `ap`, `wind`, `log`, `scn`: time axes for autopilot, wind, logging, scenario.
- `evt`: union axis used for integration boundaries.

Notes
-----
- `evt.t_us` always includes both `t0_us` and `t_end_us`.
- `evt.t_us` is strictly increasing.
"""
struct Timeline
    t0_us::UInt64
    t_end_us::UInt64

    ap::TimeAxis
    wind::TimeAxis
    log::TimeAxis
    scn::TimeAxis

    evt::TimeAxis
end

"""Build a periodic time axis.

The axis includes `t0_us` and then every `dt_us` until `t_end_us`.
If `t_end_us` is not exactly hit by the period, it will still be included
by `build_timeline` via the union axis.
"""
function periodic_axis(name::Symbol, t0_us::UInt64, t_end_us::UInt64, dt_us::UInt64)
    dt_us > 0 || throw(ArgumentError("dt_us must be > 0"))
    t_end_us >= t0_us || throw(ArgumentError("t_end_us must be >= t0_us"))

    # NOTE: allocate once; typical sizes are small.
    n = Int(div(t_end_us - t0_us, dt_us)) + 1
    t = Vector{UInt64}(undef, n)
    for i = 1:n
        t[i] = t0_us + (UInt64(i - 1) * dt_us)
    end
    return TimeAxis(name, t)
end

"""Merge and sort unique time vectors into an event axis.

This must be deterministic: ties collapse to a single time.
"""
function merge_axes(name::Symbol, axes::Vector{Vector{UInt64}})
    # Small-N deterministic merge.
    all = UInt64[]
    for a in axes
        append!(all, a)
    end
    sort!(all)
    # unique! preserves order after sort.
    unique!(all)
    return TimeAxis(name, all)
end

"""Build a full hybrid timeline.

Arguments
---------
- `t0_us`, `t_end_us` : inclusive bounds
- `dt_ap_us`, `dt_wind_us`, `dt_log_us` : periodic axes rates
- `scn_times_us` : scenario boundary times (AtTime) within the window

Design note
-----------
We treat scenario times as first-class boundaries. This is essential for proper
hybrid-system semantics and integrator fairness.
"""
function build_timeline(
    t0_us::UInt64,
    t_end_us::UInt64;
    dt_ap_us::UInt64,
    dt_wind_us::UInt64,
    dt_log_us::UInt64,
    scn_times_us::Vector{UInt64} = UInt64[],
)::Timeline
    t_end_us >= t0_us || throw(ArgumentError("t_end_us must be >= t0_us"))

    ap = periodic_axis(:autopilot, t0_us, t_end_us, dt_ap_us)
    wind = periodic_axis(:wind, t0_us, t_end_us, dt_wind_us)
    log = periodic_axis(:log, t0_us, t_end_us, dt_log_us)

    # Filter + sort scenario times; include only inside window.
    #
    # IMPORTANT: we always include `t0_us` as a scenario axis point so scenario outputs
    # (e.g., `bus.ap_cmd`, `bus.landed`) can be recorded/replayed with a valid initial
    # sample even if the first discrete scenario event occurs later.
    scn = UInt64[t for t in scn_times_us if (t >= t0_us && t <= t_end_us)]
    push!(scn, t0_us)
    sort!(scn)
    unique!(scn)
    scn_axis = TimeAxis(:scenario, scn)

    evt = merge_axes(
        :event,
        [ap.t_us, wind.t_us, log.t_us, scn_axis.t_us, UInt64[t0_us, t_end_us]],
    )

    # Ensure inclusion of endpoints.
    (evt.t_us[1] == t0_us) || error("timeline event axis must start at t0_us")
    (evt.t_us[end] == t_end_us) || error("timeline event axis must end at t_end_us")

    return Timeline(t0_us, t_end_us, ap, wind, log, scn_axis, evt)
end

"""Return true if `t_us` is on this axis.

This is used for dispatching event processing.

This is O(log N) and intended for debugging and small axis sizes. A future
optimization is to keep per-axis indices during the engine loop.
"""
function on_axis(axis::TimeAxis, t_us::UInt64)::Bool
    # binary search
    lo = 1
    hi = length(axis.t_us)
    while lo <= hi
        mid = (lo + hi) >>> 1
        v = axis.t_us[mid]
        if v == t_us
            return true
        elseif v < t_us
            lo = mid + 1
        else
            hi = mid - 1
        end
    end
    return false
end
