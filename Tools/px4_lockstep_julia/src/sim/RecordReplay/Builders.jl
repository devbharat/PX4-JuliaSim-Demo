"""Convenience builders for Option A record/replay.

This file sits *above* the low-level timeline/trace/sources primitives and provides
small helpers to reduce boilerplate in user scripts.

The goal is to keep the core components (Timeline, Bus, Traces, Sources, Engine)
minimal and deterministic, while still enabling an ergonomic user workflow.

NOTE
----
These helpers are intentionally shallow. They should never hide scheduling or
recording semantics.
"""

# We are in PX4Lockstep.Sim.RecordReplay.

"""Build a timeline for a run, automatically including scenario AtTime event boundaries.

This is a thin wrapper around `build_timeline`.

Arguments
---------
- `scenario`: scenario source instance used in the run.

Notes
-----
The scenario axis is built from:
- `event_times_us(scenario, t0_us, t_end_us)`
- and `build_timeline` will always include `t0_us` on the scenario axis.

If you pass `NullScenarioSource()`, the scenario axis will contain only `t0_us`.
"""
function build_timeline_for_run(
    t0_us::UInt64,
    t_end_us::UInt64;
    dt_ap_us::UInt64,
    dt_wind_us::UInt64,
    dt_log_us::UInt64,
    scenario::AbstractScenarioSource = NullScenarioSource(),
)::Timeline
    scn_times_us = event_times_us(scenario, t0_us, t_end_us)
    return build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        scn_times_us = scn_times_us,
    )
end
