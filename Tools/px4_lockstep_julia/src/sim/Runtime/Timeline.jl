"""Runtime.Timeline

Deterministic integer-microsecond time axes and hybrid event-timeline construction.

This is part of the canonical `Sim.Runtime.Engine` contract:
- all engine scheduling is performed in **integer microseconds** (`UInt64`)
- axes are explicit (autopilot, wind, logging, scenario, physics)
- the engine advances on the union `timeline.evt`

Scenario `AtTime` events are first-class boundaries: if your scenario intends a fault to
occur at exactly `t = 3.275 s`, that time must appear in the event axis even if it does
not align with `dt_autopilot`.

Future work
-----------
- Dynamic insertion of boundaries (hybrid systems with `When(...)` that schedule new
  `AtTime` events at runtime).
"""

"""Convert a Float64 seconds interval to integer microseconds.

The conversion must be exact for determinism: `dt_s` must be representable as an integer
number of microseconds.
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
- `phys`: optional fixed-step physics integration axis (may include only endpoints).
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

    phys::TimeAxis

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
    all = UInt64[]
    for a in axes
        append!(all, a)
    end
    sort!(all)
    unique!(all)
    return TimeAxis(name, all)
end

"""Build a full hybrid timeline.

Arguments
---------
- `t0_us`, `t_end_us` : inclusive bounds
- `dt_ap_us`, `dt_wind_us`, `dt_log_us` : periodic axes rates
- `scn_times_us` : scenario boundary times (AtTime) within the window
- `dt_phys_us` : optional fixed physics dt axis (adds boundaries only; no separate
  per-axis events)

Design note
-----------
Scenario times are treated as first-class boundaries. This is essential for proper
hybrid-system semantics and integrator fairness.
"""
function build_timeline(
    t0_us::UInt64,
    t_end_us::UInt64;
    dt_ap_us::UInt64,
    dt_wind_us::UInt64,
    dt_log_us::UInt64,
    dt_phys_us::Union{Nothing,UInt64} = nothing,
    scn_times_us::Vector{UInt64} = UInt64[],
)::Timeline
    t_end_us >= t0_us || throw(ArgumentError("t_end_us must be >= t0_us"))

    ap = periodic_axis(:autopilot, t0_us, t_end_us, dt_ap_us)
    wind = periodic_axis(:wind, t0_us, t_end_us, dt_wind_us)
    log = periodic_axis(:log, t0_us, t_end_us, dt_log_us)

    # Filter + sort scenario times; include only inside window.
    # Always include `t0_us` so scenario outputs have a valid initial sample.
    scn = UInt64[t for t in scn_times_us if (t >= t0_us && t <= t_end_us)]
    push!(scn, t0_us)
    sort!(scn)
    unique!(scn)
    scn_axis = TimeAxis(:scenario, scn)

    # Optional physics axis. If not specified, this becomes a degenerate axis with
    # only endpoints (and therefore does not add boundaries beyond the others).
    phys_axis = if dt_phys_us === nothing
        TimeAxis(:physics, UInt64[t0_us, t_end_us])
    else
        periodic_axis(:physics, t0_us, t_end_us, dt_phys_us)
    end

    evt = merge_axes(
        :event,
        [
            ap.t_us,
            wind.t_us,
            log.t_us,
            scn_axis.t_us,
            phys_axis.t_us,
            UInt64[t0_us, t_end_us],
        ],
    )

    (evt.t_us[1] == t0_us) || error("timeline event axis must start at t0_us")
    (evt.t_us[end] == t_end_us) || error("timeline event axis must end at t_end_us")

    return Timeline(t0_us, t_end_us, ap, wind, log, scn_axis, phys_axis, evt)
end

"""Return true if `t_us` is on this axis.

This is O(log N) and intended for debugging and small axis sizes.
"""
function on_axis(axis::TimeAxis, t_us::UInt64)::Bool
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

# -----------------------------------------------------------------------------
# Scenario boundary discovery protocol
# -----------------------------------------------------------------------------

"""Return discrete scenario boundary times within `[t0_us, t_end_us]`.

This is a protocol hook used by `build_timeline_for_run`.

Default behavior is "no scenario boundaries".
Scenario sources are expected to extend this method.
"""
function event_times_us(_scenario, _t0_us::UInt64, _t_end_us::UInt64)
    return UInt64[]
end

"""Build a timeline for a run, automatically including scenario boundaries.

This is a thin wrapper around `build_timeline` that queries `event_times_us(scenario, ...)`.
"""
function build_timeline_for_run(
    t0_us::UInt64,
    t_end_us::UInt64;
    dt_ap_us::UInt64,
    dt_wind_us::UInt64,
    dt_log_us::UInt64,
    dt_phys_us::Union{Nothing,UInt64} = nothing,
    scenario = nothing,
)::Timeline
    scn_times_us = event_times_us(scenario, t0_us, t_end_us)
    return build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scn_times_us = scn_times_us,
    )
end

export TimeAxis, Timeline
export dt_to_us, periodic_axis, merge_axes, build_timeline, build_timeline_for_run, on_axis
export event_times_us
