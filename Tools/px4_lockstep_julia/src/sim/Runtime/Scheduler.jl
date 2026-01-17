"""Runtime.Scheduler

Stateful iterator over a `Timeline`.

This separates:
- `Timeline`: pure data structure (axes vectors)
- `Scheduler`: stateful logic that walks the event union axis and determines which
  sub-axes are due at each boundary.

Why this exists
---------------
The engine previously used a mix of `searchsortedfirst` on `timeline.evt` and
per-axis cursors. That works, but keeping the logic here makes:
- boundary membership checking O(1)
- boundary due-set computation explicit and testable
- drift harder (the scheduler is the only place that defines what "due" means)

Contract
--------
- All times are integer microseconds (`UInt64`).
- The scheduler walks `timeline.evt.t_us` in strictly increasing order.
- For each axis, the scheduler maintains an index pointing at the *next* pending
  time on that axis.
"""

# NOTE: `BoundaryEvent` is defined in BoundaryProtocol.jl.

"""Stateful scheduler for a `Timeline`."""
mutable struct Scheduler
    timeline::Timeline
    evt_idx::Int
    ap_idx::Int
    wind_idx::Int
    log_idx::Int
    scn_idx::Int
    phys_idx::Int
end

"""Return the first index in `axis.t_us` that is >= `t0_us`."""
@inline function _start_idx(axis::TimeAxis, t0_us::UInt64)::Int
    i = searchsortedfirst(axis.t_us, t0_us)
    return i
end

"""Construct a scheduler positioned at `timeline.t0_us`."""
function Scheduler(timeline::Timeline)
    evt = timeline.evt.t_us
    isempty(evt) && error("timeline.evt is empty")
    (evt[1] == timeline.t0_us) || error("timeline.evt must start at t0_us")

    return Scheduler(
        timeline,
        1,
        _start_idx(timeline.ap, timeline.t0_us),
        _start_idx(timeline.wind, timeline.t0_us),
        _start_idx(timeline.log, timeline.t0_us),
        _start_idx(timeline.scn, timeline.t0_us),
        _start_idx(timeline.phys, timeline.t0_us),
    )
end

@inline function current_us(s::Scheduler)::UInt64
    return s.timeline.evt.t_us[s.evt_idx]
end

@inline function has_next(s::Scheduler)::Bool
    return s.evt_idx < length(s.timeline.evt.t_us)
end

@inline function next_us(s::Scheduler)::UInt64
    if !has_next(s)
        return s.timeline.evt.t_us[end]
    end
    return s.timeline.evt.t_us[s.evt_idx+1]
end

"""Compute which sub-axes are due at the current boundary time.

This does not mutate the scheduler.
"""
function boundary_event(s::Scheduler)::BoundaryEvent
    t_us = current_us(s)

    ap = s.timeline.ap.t_us
    wind = s.timeline.wind.t_us
    log = s.timeline.log.t_us
    scn = s.timeline.scn.t_us
    phys = s.timeline.phys.t_us

    due_ap = (s.ap_idx <= length(ap) && ap[s.ap_idx] == t_us)
    due_wind = (s.wind_idx <= length(wind) && wind[s.wind_idx] == t_us)
    due_log = (s.log_idx <= length(log) && log[s.log_idx] == t_us)
    due_scn = (s.scn_idx <= length(scn) && scn[s.scn_idx] == t_us)
    due_phys = (s.phys_idx <= length(phys) && phys[s.phys_idx] == t_us)

    return BoundaryEvent(t_us, due_scn, due_wind, due_ap, due_log, due_phys)
end

"""Consume the current boundary event by advancing axis indices for any due axes."""
function consume_boundary!(s::Scheduler, ev::BoundaryEvent)
    # Assert scheduler/ev alignment.
    current_us(s) == ev.time_us || error("consume_boundary!: event time mismatch")

    ev.due_ap && (s.ap_idx += 1)
    ev.due_wind && (s.wind_idx += 1)
    ev.due_log && (s.log_idx += 1)
    ev.due_scn && (s.scn_idx += 1)
    ev.due_phys && (s.phys_idx += 1)

    return nothing
end

"""Advance the scheduler to the next event-axis boundary."""
function advance_evt!(s::Scheduler)
    has_next(s) || return false
    s.evt_idx += 1
    return true
end

export Scheduler,
    current_us, has_next, next_us, boundary_event, consume_boundary!, advance_evt!
