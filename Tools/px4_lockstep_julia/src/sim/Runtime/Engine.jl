"""Runtime.Engine

Canonical hybrid simulation engine.

This is the *only* authoritative run loop intended to remain long-term.

Core responsibilities
---------------------
- advance time on a deterministic integer-microsecond `Timeline`
- step discrete-time sources at their axis boundaries
- integrate the continuous plant between boundaries (fixed-step or adaptive)
- (optional) record bus/plant streams for replay/verification

Key design rules
----------------
- **No RNG in the plant RHS**; all randomness is sampled in sources.
- **All schedule decisions use `UInt64` microsecond clocks**.
- Inputs are **sample-and-hold** between boundaries.

Protocols
---------
The engine uses duck-typed protocols:
- `update!(source, bus, plant_state, t_us)` is called at boundaries.
- `record!(recorder, name, t_us, value)` is called at record points.
- `finalize!(recorder)` is called at end of `run!`.

Concrete implementations live under `Sim.Sources` and `Sim.Recording`.
"""

using ..Types: Vec3, quat_rotate_inv
using ..RigidBody: RigidBodyState
using ..Vehicles: ActuatorCommand, sanitize
using ..Plant: PlantInput, PlantOutputs, PlantState
using ..Integrators: AbstractIntegrator, step_integrator, last_stats, reset!

import ..plant_outputs
import ..plant_project
import ..plant_on_autopilot_tick

import ..Logging

export EngineMode, EngineConfig, Engine, EngineStats, EngineOutputs
export run!, step_to_next_event!, process_events_at!
export update!, record!, finalize!
export plant_record_engine, plant_replay_engine


############################
# Protocol entrypoints
############################

"""Protocol: update a discrete-time source at a boundary.

Sources (scenario, wind, estimator, autopilot) implement this method.

The default fallback is a loud error to avoid silent mis-wiring.
"""
function update!(src, bus::SimBus, plant_state, t_us::UInt64)
    error("No Runtime.update! method for source type $(typeof(src))")
end

"""No-op source."""
@inline update!(::Nothing, bus::SimBus, plant_state, t_us::UInt64) = nothing

"""Protocol: record a stream sample.

Recorders implement `record!(recorder, name, t_us, value)`.

The default fallback is a loud error; pass `nothing` for a no-op recorder.
"""
function record!(recorder, name::Symbol, t_us::UInt64, value)
    error("No Runtime.record! method for recorder type $(typeof(recorder))")
end

"""No-op recorder."""
@inline record!(::Nothing, name::Symbol, t_us::UInt64, value) = nothing

"""Protocol: finalize a recorder after a run."""
function finalize!(recorder)
    error("No Runtime.finalize! method for recorder type $(typeof(recorder))")
end

"""No-op recorder finalization."""
@inline finalize!(::Nothing) = nothing


############################
# Log sink normalization
############################

"""Normalize `log_sinks` to a Vector{Logging.AbstractLogSink} for deterministic iteration."""
function _normalize_log_sinks(log_sinks)::Vector{Logging.AbstractLogSink}
    if log_sinks === nothing
        return Logging.AbstractLogSink[]
    elseif log_sinks isa Logging.AbstractLogSink
        return Logging.AbstractLogSink[log_sinks]
    elseif log_sinks isa Tuple || log_sinks isa AbstractVector
        v = Logging.AbstractLogSink[]
        for s in log_sinks
            s isa Logging.AbstractLogSink ||
                error("log_sinks element must be AbstractLogSink, got " * string(typeof(s)))
            push!(v, s)
        end
        return v
    else
        error(
            "log_sinks must be nothing, an AbstractLogSink, or a tuple/vector of AbstractLogSink; got " *
            string(typeof(log_sinks)),
        )
    end
end


############################
# Engine config + bookkeeping
############################

@enum EngineMode begin
    MODE_LIVE
    MODE_RECORD
    MODE_REPLAY
end

Base.@kwdef struct EngineConfig
    mode::EngineMode = MODE_LIVE

    """If true, evaluate `plant_outputs` at boundaries to populate bus battery telemetry."""
    enable_derived_outputs::Bool = true

    """If true, record scenario outputs on the dense `timeline.evt` axis.

This is recommended if scenarios can change faults at non-AtTime boundaries via
hybrid logic (`When(...)`), because the only safe replay of such transitions is
an event-axis stream.
"""
    record_faults_evt::Bool = true

    """If true, record estimator output (`:est`) on the autopilot axis."""
    record_estimator::Bool = false

    """Development-time validation checks for determinism invariants."""
    validator::EngineValidator = EngineValidator()

    """If true, sanitize actuator commands at the engine boundary."""
    sanitize_cmd::Bool = true

    """If true, invalid actuator commands throw instead of being silently clamped."""
    strict_cmd::Bool = false
end

"""Lightweight runtime stats (non-authoritative, but useful for debugging)."""
Base.@kwdef mutable struct EngineStats
    # Boundary processing stats
    n_boundaries::Int = 0
    last_boundary_us::UInt64 = 0

    # Integration interval stats
    n_intervals::Int = 0
    last_interval_us::UInt64 = 0
    last_integrator_stats::Any = nothing
end

"""Cached algebraic outputs from the plant model."""
Base.@kwdef mutable struct EngineOutputs
    # Cached plant_outputs(...) result for the current boundary (if computed).
    plant_y::Any = nothing

    derived_valid::Bool = false
end


############################
# Engine struct
############################

mutable struct Engine{PS,DF,I,AP,W,SC,ES,TL,R}
    cfg::EngineConfig
    timeline::Timeline
    sched::Scheduler

    # authoritative time (redundant with sched, but convenient)
    t_us::UInt64
    t_s::Float64

    # deterministic boundary counter (0-based index into timeline.evt)
    step::Int

    # state
    bus::SimBus
    plant::PS

    # continuous model + integrator
    dynfun::DF
    integrator::I

    # discrete sources
    autopilot::AP
    wind::W
    scenario::SC
    estimator::ES

    # optional boundary-time telemetry hooks
    telemetry::TL

    # optional recorder
    recorder::R

    # optional structured logs (CSV sink etc.)
    log_sinks::Vector{Logging.AbstractLogSink}

    outputs::EngineOutputs
    stats::EngineStats

    # Capability flags (computed once at construction).
    has_outputs::Bool
    has_project::Bool
    has_ap_tick::Bool
end

function Engine(
    cfg::EngineConfig,
    timeline::Timeline,
    bus::SimBus,
    plant0,
    dynfun,
    integrator::AbstractIntegrator,
    autopilot,
    wind;
    scenario = nothing,
    estimator = nothing,
    telemetry = NullTelemetry(),
    recorder = nothing,
    log_sinks = nothing,
)
    sched = Scheduler(timeline)
    t_us = current_us(sched)
    t_s = Float64(t_us) * 1e-6

    step = Int(sched.evt_idx) - 1

    # Reset bus time + schema invariants.
    reset_bus!(bus, t_us)

    ls = _normalize_log_sinks(log_sinks)

    has_outputs =
        cfg.enable_derived_outputs &&
        hasmethod(plant_outputs, Tuple{typeof(dynfun),Float64,typeof(plant0),PlantInput})
    has_project = hasmethod(plant_project, Tuple{typeof(dynfun),typeof(plant0)})
    has_ap_tick = hasmethod(
        plant_on_autopilot_tick,
        Tuple{typeof(dynfun),typeof(plant0),ActuatorCommand},
    )

    return Engine(
        cfg,
        timeline,
        sched,
        t_us,
        t_s,
        step,
        bus,
        plant0,
        dynfun,
        integrator,
        autopilot,
        wind,
        scenario,
        estimator,
        telemetry,
        recorder,
        ls,
        EngineOutputs(),
        EngineStats(),
        has_outputs,
        has_project,
        has_ap_tick,
    )
end

"""Keyword-based outer constructor.

This is the intended public construction surface for building an engine.

Keeping this as a keyword constructor avoids fragile positional ordering and makes
call sites (examples/tests) self-documenting.
"""
function Engine(
    cfg::EngineConfig;
    timeline::Timeline,
    plant0,
    dynfun,
    integrator::AbstractIntegrator,
    autopilot,
    wind,
    scenario = nothing,
    estimator = nothing,
    telemetry = NullTelemetry(),
    recorder = nothing,
    log_sinks = nothing,
    bus::SimBus = SimBus(time_us = timeline.t0_us),
)
    return Engine(
        cfg,
        timeline,
        bus,
        plant0,
        dynfun,
        integrator,
        autopilot,
        wind;
        scenario = scenario,
        estimator = estimator,
        telemetry = telemetry,
        recorder = recorder,
        log_sinks = log_sinks,
    )
end


############################
# Convenience constructors
############################

"""Construct an engine for a record run.

This constructor is intentionally minimal: sources (live or replay) must be provided
explicitly so `Runtime` does not depend on `Sources` or `Recording` modules.
"""
function plant_record_engine(; kwargs...)
    # Record runs are where determinism matters most; fail fast on bad actuator packets.
    cfg = EngineConfig(mode = MODE_RECORD, strict_cmd = true)
    return Engine(cfg; kwargs...)
end

"""Construct an engine for a replay run."""
function plant_replay_engine(; kwargs...)
    cfg = EngineConfig(mode = MODE_REPLAY)
    return Engine(cfg; kwargs...)
end


############################
# Boundary processing
############################

"""Process all discrete sources that are due at the current boundary time.

This is intentionally the *only* place where discrete ordering semantics are defined.
"""
function process_events_at!(sim::Engine)
    # Current boundary is defined by the scheduler.
    ev = boundary_event(sim.sched)
    prev_wind_dist = sim.bus.wind_dist_ned

    # Make time authoritative + consistent everywhere.
    sim.t_us = ev.time_us
    sim.t_s = Float64(sim.t_us) * 1e-6
    sim.step = Int(sim.sched.evt_idx) - 1
    sim.bus.time_us = sim.t_us

    # Fail fast on determinism/scheduler invariants.
    validate_boundary!(sim.cfg.validator, sim, ev)

    # Canonical stage order. This loop makes reordering difficult to do accidentally.
    for stage in CANONICAL_STAGE_ORDER
        if stage === :scenario
            # Scenario first: can set bus-level faults and high-level commands.
            update!(sim.scenario, sim.bus, sim.plant, sim.t_us)

            # If the wind axis is not due, apply any change in the scenario wind
            # disturbance immediately so the held wind sample is consistent.
            if !ev.due_wind
                sim.bus.wind_ned =
                    sim.bus.wind_ned + (sim.bus.wind_dist_ned - prev_wind_dist)
            end

            # Optionally record dense scenario streams on evt axis.
            if sim.cfg.mode == MODE_RECORD && sim.cfg.record_faults_evt
                record!(sim.recorder, :faults_evt, sim.t_us, sim.bus.faults)
                record!(sim.recorder, :ap_cmd_evt, sim.t_us, sim.bus.ap_cmd)
                record!(sim.recorder, :landed_evt, sim.t_us, sim.bus.landed)
                record!(sim.recorder, :wind_dist_evt, sim.t_us, sim.bus.wind_dist_ned)
            end

            # Also record on the scenario axis when due (small stream for static scenarios).
            if sim.cfg.mode == MODE_RECORD && ev.due_scn
                record!(sim.recorder, :faults, sim.t_us, sim.bus.faults)
                record!(sim.recorder, :ap_cmd, sim.t_us, sim.bus.ap_cmd)
                record!(sim.recorder, :landed, sim.t_us, sim.bus.landed)
                record!(sim.recorder, :wind_dist, sim.t_us, sim.bus.wind_dist_ned)
            end

        elseif stage === :wind
            if ev.due_wind
                update!(sim.wind, sim.bus, sim.plant, sim.t_us)

                # Apply any scenario-requested wind disturbance after the base wind
                # source updates the sample.
                sim.bus.wind_ned = sim.bus.wind_ned + sim.bus.wind_dist_ned
                if sim.cfg.mode == MODE_RECORD
                    record!(sim.recorder, :wind_ned, sim.t_us, sim.bus.wind_ned)
                end
            end

        elseif stage === :derived_outputs
            # Derived outputs (battery telemetry) *after* faults/wind and before autopilot.
            if sim.has_outputs
                u = PlantInput(
                    cmd = sim.bus.cmd,
                    wind_ned = sim.bus.wind_ned,
                    faults = sim.bus.faults,
                )

                y = plant_outputs(sim.dynfun, sim.t_s, sim.plant, u)
                y isa PlantOutputs || error(
                    "plant_outputs must return Plant.PlantOutputs; got " *
                    string(typeof(y)),
                )

                sim.outputs.plant_y = y
                sim.outputs.derived_valid = true

                # Battery telemetry is the most important derived output: PX4 consumes it.
                # Phase 5.3: prefer the full per-battery vector when available.
                if y.battery_statuses !== nothing
                    bats = y.battery_statuses
                    nb = length(bats)
                    if length(sim.bus.batteries) != nb
                        error(
                            "bus.batteries length mismatch: bus has $(length(sim.bus.batteries)) " *
                            "but plant_outputs returned $(nb) batteries",
                        )
                    end
                    for i = 1:nb
                        sim.bus.batteries[i] = bats[i]
                    end
                end

                # Env cache for sinks/telemetry (explicit schema; NaN means 'missing').
                rho = y.rho_kgm3
                if isfinite(rho)
                    sim.bus.env = EnvSample(rho_kgm3 = rho, temp_k = sim.bus.env.temp_k)
                end
                temp = y.temp_k
                if isfinite(temp)
                    sim.bus.env = EnvSample(rho_kgm3 = sim.bus.env.rho_kgm3, temp_k = temp)
                end
            else
                sim.outputs.plant_y = nothing
                sim.outputs.derived_valid = false
            end

        elseif stage === :estimator
            if ev.due_ap
                update!(sim.estimator, sim.bus, sim.plant, sim.t_us)
                if sim.cfg.mode == MODE_RECORD
                    if sim.cfg.record_estimator
                        record!(sim.recorder, :est, sim.t_us, sim.bus.est)
                    end
                end
            end

        elseif stage === :telemetry
            # Optional deterministic hooks for inspection/metrics.
            # Runs at autopilot ticks so it can observe exactly what PX4 sees.
            if ev.due_ap
                telemetry!(sim.telemetry, sim.bus, sim.plant, sim.t_us)
            end

        elseif stage === :autopilot
            if ev.due_ap
                # Autopilot consumes bus.est/batteries and produces bus.cmd.
                update!(sim.autopilot, sim.bus, sim.plant, sim.t_us)

                # Defensive boundary validation: the plant integrator expects finite,
                # range-bounded actuator commands. Record runs should fail fast.
                if sim.cfg.sanitize_cmd
                    sim.bus.cmd = sanitize(sim.bus.cmd; strict = sim.cfg.strict_cmd)
                end
                if sim.cfg.mode == MODE_RECORD
                    record!(sim.recorder, :cmd, sim.t_us, sim.bus.cmd)
                end
            end

        elseif stage === :plant_discontinuities
            if ev.due_ap
                # Boundary-time plant discontinuities at autopilot ticks.
                if sim.has_ap_tick
                    sim.plant = plant_on_autopilot_tick(sim.dynfun, sim.plant, sim.bus.cmd)
                end
            end

        elseif stage === :logging
            if ev.due_log
                # Boundary-time logs are *pre-step* snapshots.
                record!(sim.recorder, :plant, sim.t_us, sim.plant)

                # Always record battery on the log axis so tier0 recordings are schema-stable,
                # even for simplified dynamics models that do not implement plant_outputs(...).
                record!(sim.recorder, :battery, sim.t_us, sim.bus.batteries[1])

                # Phase 5.3: record the full battery vector as a snapshot so recorded
                # values are not aliased through the mutable bus vector.
                record!(sim.recorder, :batteries, sim.t_us, copy(sim.bus.batteries))

                _emit_logs_to_sinks!(sim)
            end

        else
            error("Unknown boundary stage: $stage")
        end
    end

    # Update boundary stats.
    if sim.stats.n_boundaries == 0
        sim.stats.last_boundary_us = sim.t_us
    else
        sim.stats.last_boundary_us = sim.t_us
    end
    sim.stats.n_boundaries += 1

    # Consume axis membership for this boundary.
    consume_boundary!(sim.sched, ev)

    return nothing
end


############################
# Logging sinks
############################

@inline _rb_state(x::RigidBodyState) = x
@inline _rb_state(x::PlantState) = x.rb

@inline function _rb_state(x)
    if hasproperty(x, :rb)
        return getproperty(x, :rb)
    end
    error(
        "Plant state must expose an `rb` field to emit rigid-body logs. Got: " *
        string(typeof(x)),
    )
end

@inline function _rotor_omega_tuple(plant::PlantState)::NTuple{12,Float64}
    ω = plant.rotor_ω
    n = length(ω)
    return ntuple(i -> i <= n ? Float64(ω[i]) : NaN, 12)
end

@inline _rotor_omega_tuple(::Any)::NTuple{12,Float64} = Logging.NAN_ROTOR_TUPLE

@inline _rotor_thrust_tuple(::Nothing)::NTuple{12,Float64} = Logging.NAN_ROTOR_TUPLE

@inline function _rotor_thrust_tuple(y::PlantOutputs)::NTuple{12,Float64}
    rot = y.rotors
    rot === nothing && return Logging.NAN_ROTOR_TUPLE
    th = rot.thrust_n
    n = length(th)
    return ntuple(i -> i <= n ? Float64(th[i]) : NaN, 12)
end

@inline _rotor_thrust_tuple(::Any)::NTuple{12,Float64} = Logging.NAN_ROTOR_TUPLE

@inline function _to_ntuple3_float(v)
    return (Float64(v[1]), Float64(v[2]), Float64(v[3]))
end

"""Typed autopilot telemetry for structured logging.

Autopilot sources should populate this contract at the source boundary so the
runtime log schema is stable and missing data is explicit (NaNs / -1).
"""
Base.@kwdef struct AutopilotTelemetry
    pos_sp::NTuple{3,Float64} = (NaN, NaN, NaN)
    vel_sp::NTuple{3,Float64} = (NaN, NaN, NaN)
    acc_sp::NTuple{3,Float64} = (NaN, NaN, NaN)
    yaw_sp::Float64 = NaN
    yawspeed_sp::Float64 = NaN
    nav_state::Int32 = Int32(-1)
    arming_state::Int32 = Int32(-1)
    mission_seq::Int32 = Int32(0)
    mission_count::Int32 = Int32(0)
    mission_finished::Int32 = Int32(0)
end

"""Protocol hook: extract typed autopilot telemetry.

Default implementation returns missing values. Live sources should override.
"""
@inline autopilot_telemetry(::Any) = AutopilotTelemetry()

"""Emit boundary-time logs to all configured log sinks.

This is explicitly treated as a *side-effect only* hook: it must not mutate simulation state.
"""
function _emit_logs_to_sinks!(sim::Engine)
    isempty(sim.log_sinks) && return

    rb = _rb_state(sim.plant)

    wind_ned = sim.bus.wind_ned
    v_air_ned = wind_ned - rb.vel_ned
    air_vel_body = _to_ntuple3_float(quat_rotate_inv(rb.q_bn, v_air_ned))

    rotor_omega = _rotor_omega_tuple(sim.plant)
    rotor_thrust = _rotor_thrust_tuple(sim.outputs.plant_y)

    ap_tel = autopilot_telemetry(sim.autopilot)

    batt = sim.bus.batteries[1]
    for sink in sim.log_sinks
        Logging.log!(
            sink,
            sim.t_s,
            rb,
            sim.bus.cmd;
            time_us = sim.t_us,
            wind_ned = wind_ned,
            rho = sim.bus.env.rho_kgm3,
            air_vel_body = air_vel_body,
            battery = batt,
            rotor_omega = rotor_omega,
            rotor_thrust = rotor_thrust,
            pos_sp = ap_tel.pos_sp,
            vel_sp = ap_tel.vel_sp,
            acc_sp = ap_tel.acc_sp,
            yaw_sp = ap_tel.yaw_sp,
            yawspeed_sp = ap_tel.yawspeed_sp,
            nav_state = ap_tel.nav_state,
            arming_state = ap_tel.arming_state,
            mission_seq = ap_tel.mission_seq,
            mission_count = ap_tel.mission_count,
            mission_finished = ap_tel.mission_finished,
        )
    end

    return nothing
end


############################
# Integration stepping
############################

"""Integrate from current time to the next event boundary."""
function step_to_next_event!(sim::Engine)
    has_next(sim.sched) || return false

    t_us = current_us(sim.sched)
    t_us == sim.t_us ||
        error("Engine/scheduler time mismatch: sched=$t_us engine=$(sim.t_us)")

    next_t_us = next_us(sim.sched)
    dt_us = next_t_us - t_us
    dt_s = Float64(dt_us) * 1e-6

    # Integrate the plant over [t, t+dt] with held inputs.
    u = PlantInput(cmd = sim.bus.cmd, wind_ned = sim.bus.wind_ned, faults = sim.bus.faults)

    reset!(sim.integrator)
    sim.plant = step_integrator(sim.integrator, sim.dynfun, sim.t_s, sim.plant, u, dt_s)

    # Optional post-step projection into physical bounds.
    if sim.has_project
        sim.plant = plant_project(sim.dynfun, sim.plant)
    end

    # Advance scheduler to the new boundary.
    advance_evt!(sim.sched) || error("advance_evt! failed unexpectedly")

    # Advance authoritative time.
    sim.t_us = current_us(sim.sched)
    sim.t_s = Float64(sim.t_us) * 1e-6
    sim.step = Int(sim.sched.evt_idx) - 1

    # Bookkeeping.
    sim.stats.n_intervals += 1
    sim.stats.last_interval_us = dt_us
    sim.stats.last_integrator_stats = last_stats(sim.integrator)

    return true
end


############################
# Run loop
############################

"""Run the engine to `timeline.t_end_us`.

Returns the mutated engine for convenience.
"""
function run!(sim::Engine)
    # Initial boundary processing.
    process_events_at!(sim)

    while has_next(sim.sched)
        ok = step_to_next_event!(sim)
        ok || break
        process_events_at!(sim)
    end

    finalize!(sim.recorder)

    # Ensure log sinks flush and close deterministically.
    for s in sim.log_sinks
        Logging.close!(s)
    end

    return sim
end
