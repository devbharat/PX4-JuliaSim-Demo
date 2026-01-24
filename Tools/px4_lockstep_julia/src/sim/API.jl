"""High-level simulation API.

Why this exists
---------------
The internal architecture intentionally has many composable pieces:

- `Runtime.Engine` (one canonical run loop)
- `Runtime.Timeline` (event axes)
- `Sources.*` (live or replay sources)
- `Recording.*` (recorders and traces)
- `PlantModels.*` (continuous-time plant RHS)
- `Integrators.*` (fixed-step or adaptive)

That modularity is good for correctness and testing, but users should not have to
learn the plumbing to run common workflows.

This file defines a small set of **stable, user-facing entrypoints** that:
- build a canonical `Runtime.Engine`
- run it
- (optionally) return or save a recording

Backwards compatibility
-----------------------
Backwards compatibility with removed legacy engines is not a goal.
This API is the intended replacement.
"""

import Base.CoreLogging: @warn

# Note: This file is included into `module Sim`, so these are `PX4Lockstep.Sim.*` symbols.

"""Build and run the canonical engine.

This is the single entrypoint that all other workflows should use.

Keyword args are intentionally explicit. For ergonomic wrappers (PX4 Iris, etc.),
see the examples under `examples/`.

Parameters
----------
- `mode`: `:live | :record | :replay`
- `timeline`: `Runtime.Timeline`
- `plant0`: initial plant state (`RigidBodyState` or `PlantState`)
- `dynfun`: continuous-time dynamics functor
- `integrator`: integrator instance (Euler/RK4/RK23/RK45)
- `autopilot`, `wind`, `scenario`, `estimator`: source instances (live or replay)
- `recorder`: recorder sink (e.g. `Recording.InMemoryRecorder()`), required for `:record`

Returns
-------
The mutated `Runtime.Engine` after `run!`.
"""
function simulate(;
    mode::Symbol = :live,
    timeline,
    plant0,
    dynfun,
    integrator,
    autopilot,
    wind,
    scenario = Sources.NullScenarioSource(),
    estimator = Sources.NullEstimatorSource(),
    telemetry = Runtime.NullTelemetry(),
    recorder = nothing,
    log_sinks = nothing,
    enable_derived_outputs::Bool = true,
    record_faults_evt::Bool = true,
    record_estimator::Bool = false,
    sanitize_cmd::Bool = true,
    strict_cmd::Bool = (mode === :record),
    validator = Runtime.EngineValidator(),
    strict_lockstep_rates::Bool = true,
)
    emode = if mode === :live
        Runtime.MODE_LIVE
    elseif mode === :record
        Runtime.MODE_RECORD
    elseif mode === :replay
        Runtime.MODE_REPLAY
    else
        error("simulate: unknown mode=$(mode) (expected :live|:record|:replay)")
    end

    cfg = Runtime.EngineConfig(
        mode = emode,
        enable_derived_outputs = enable_derived_outputs,
        record_faults_evt = record_faults_evt,
        record_estimator = record_estimator,
        validator = validator,
        sanitize_cmd = sanitize_cmd,
        strict_cmd = strict_cmd,
    )

    if emode == Runtime.MODE_RECORD && recorder === nothing
        error("simulate(mode=:record) requires a recorder")
    end

    _validate_lockstep_rates(autopilot, timeline; strict = strict_lockstep_rates)

    # Phase 5.3: size the bus battery vector from the plant initial state when possible.
    n_batteries = 1
    try
        if hasproperty(plant0, :power)
            p = getproperty(plant0, :power)
            if hasproperty(p, :soc)
                n_batteries = length(getproperty(p, :soc))
            end
        end
    catch
        n_batteries = 1
    end

    bus = Runtime.SimBus(time_us = timeline.t0_us, n_batteries = n_batteries)

    eng = Runtime.Engine(
        cfg;
        timeline = timeline,
        bus = bus,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integrator,
        autopilot = autopilot,
        wind = wind,
        scenario = scenario,
        estimator = estimator,
        telemetry = telemetry,
        recorder = recorder,
        log_sinks = log_sinks,
    )

    Runtime.run!(eng)
    return eng
end

function _axis_dt_us(axis::Runtime.TimeAxis)
    t = axis.t_us
    length(t) < 2 && return nothing
    dt_us = t[2] - t[1]
    for i = 3:length(t)
        (t[i] - t[i-1]) == dt_us ||
            error("axis $(axis.name) is non-uniform; cannot compute dt")
    end
    return dt_us
end

function _validate_lockstep_rates(autopilot, timeline; strict::Bool)
    autopilot isa Sources.LiveAutopilotSource || return nothing
    ap = autopilot.ap
    max_hz = Autopilots.max_internal_rate_hz(ap)
    max_hz === nothing && return nothing

    dt_us = _axis_dt_us(timeline.ap)
    dt_us === nothing && return nothing
    dt_s = Float64(dt_us) * 1e-6
    min_dt_s = 1.0 / Float64(max_hz)

    if dt_s > min_dt_s + 1e-12
        msg = "autopilot cadence dt=$(dt_s)s exceeds max_internal_rate_hz=$(max_hz) (min dt=$(min_dt_s)s)"
        if strict
            throw(ArgumentError(msg))
        else
            @warn msg
        end
    end

    # Phase 7: validate uORB injection schedule vs PX4 step cadence.
    #
    # If the simulator publishes certain uORB topics at fixed periods, the PX4
    # step cadence must be fast enough (and align as an integer divisor) so that
    # every scheduled publish time is representable on the timeline.
    req_dt_us = Autopilots.recommended_step_dt_us(ap)
    if req_dt_us !== nothing && dt_us > req_dt_us
        msg = "autopilot cadence dt=$(dt_s)s (dt_us=$(dt_us)) is too slow for uORB injection schedule (min period $(req_dt_us)us)"
        if strict
            throw(ArgumentError(msg))
        else
            @warn msg
        end
    end

    for period_us in Autopilots.injection_periods_us(ap)
        if period_us % dt_us != 0
            msg = "uORB injection period $(period_us)us is not a multiple of autopilot cadence dt_us=$(dt_us); scheduled publishes will be skipped (effective period becomes lcm)"
            if strict
                throw(ArgumentError(msg))
            else
                @warn msg
            end
        end
    end
    return nothing
end

"""Convenience wrapper: live run."""
run_live_px4(; kwargs...) = simulate(; mode = :live, kwargs...)

"""Convenience wrapper: record run."""
record_live_px4(; kwargs...) = simulate(; mode = :record, kwargs...)

"""Convenience wrapper: replay run."""
replay_recording(; kwargs...) = simulate(; mode = :replay, kwargs...)

# TODO (Phase 4 UX): provide `replay_recording(path_or_recording; ...)` that:
# - loads a Tier0Recording
# - constructs replay sources automatically
# - runs the engine
# - returns an Engine and comparison summary


"""Generic integrator comparison on a Tier-0 recording.

This is a thin ergonomic alias for `compare_integrators_recording`.

Typical use:

    rows = Sim.compare_integrators(
        recording = rec,
        dynfun = dynfun,
        solvers = [:RK4, :RK23],
        reference_integrator = Sim.iris_reference_integrator(),
        make_integrator = Sim.iris_integrator,
    )

For an end-to-end Iris workflow (record + replay sweep), see
`compare_integrators_iris_mission`.
"""
compare_integrators(; kwargs...) = compare_integrators_recording(; kwargs...)
