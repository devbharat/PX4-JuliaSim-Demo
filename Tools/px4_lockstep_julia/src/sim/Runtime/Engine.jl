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
using ..Vehicles: ActuatorCommand
using ..Powertrain: BatteryStatus
using ..Plant: PlantInput
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
            s isa Logging.AbstractLogSink || error("log_sinks element must be AbstractLogSink, got " * string(typeof(s)))
            push!(v, s)
        end
        return v
    else
        error("log_sinks must be nothing, an AbstractLogSink, or a tuple/vector of AbstractLogSink; got " * string(typeof(log_sinks)))
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

    # Boundary-time battery telemetry (updated from plant_y when available).
    battery::BatteryStatus = BatteryStatus()

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

    # Reset bus time + schema invariants.
    reset_bus!(bus, t_us)

    ls = _normalize_log_sinks(log_sinks)

    return Engine(
        cfg,
        timeline,
        sched,
        t_us,
        t_s,
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
    )
end

"""Keyword-based outer constructor.

This is the intended public construction surface for building an engine.

Keeping this as a keyword constructor avoids fragile positional ordering and makes
call sites (examples/tests) self-documenting.
"""
function Engine(cfg::EngineConfig; 
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

This constructor is intentionally minimal: you must provide the sources (live or replay)
explicitly so `Runtime` does not depend on `Sources` or `Recording` modules.
"""
function plant_record_engine(; kwargs...)
    cfg = EngineConfig(mode = MODE_RECORD)
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

    # Make time authoritative + consistent everywhere.
    sim.t_us = ev.time_us
    sim.t_s = Float64(sim.t_us) * 1e-6
    sim.bus.time_us = sim.t_us

    # Fail fast on determinism/scheduler invariants.
    validate_boundary!(sim.cfg.validator, sim, ev)

    # Canonical stage order. This loop makes reordering difficult to do accidentally.
    for stage in CANONICAL_STAGE_ORDER
        if stage === :scenario
            # Scenario first: can set bus-level faults and high-level commands.
            update!(sim.scenario, sim.bus, sim.plant, sim.t_us)

            # Optionally record dense scenario streams on evt axis.
            if sim.cfg.mode == MODE_RECORD && sim.cfg.record_faults_evt
                record!(sim.recorder, :faults_evt, sim.t_us, sim.bus.faults)
                record!(sim.recorder, :ap_cmd_evt, sim.t_us, sim.bus.ap_cmd)
                record!(sim.recorder, :landed_evt, sim.t_us, sim.bus.landed)
            end

            # Also record on the scenario axis when due (small stream for static scenarios).
            if sim.cfg.mode == MODE_RECORD && ev.due_scn
                record!(sim.recorder, :faults, sim.t_us, sim.bus.faults)
                record!(sim.recorder, :ap_cmd, sim.t_us, sim.bus.ap_cmd)
                record!(sim.recorder, :landed, sim.t_us, sim.bus.landed)
            end

        elseif stage === :wind
            if ev.due_wind
                update!(sim.wind, sim.bus, sim.plant, sim.t_us)
                if sim.cfg.mode == MODE_RECORD
                    record!(sim.recorder, :wind_ned, sim.t_us, sim.bus.wind_ned)
                end
            end

        elseif stage === :derived_outputs
            # Derived outputs (battery telemetry) *after* faults/wind and before autopilot.
            if sim.cfg.enable_derived_outputs
                u = PlantInput(cmd = sim.bus.cmd, wind_ned = sim.bus.wind_ned, faults = sim.bus.faults)

                if applicable(plant_outputs, sim.dynfun, sim.t_s, sim.plant, u)
                    y = plant_outputs(sim.dynfun, sim.t_s, sim.plant, u)
                    sim.outputs.plant_y = y
                    sim.outputs.derived_valid = true

                    # Battery telemetry is the most important derived output: PX4 consumes it.
                    if y.battery_status !== nothing
                        sim.outputs.battery = y.battery_status
                        sim.bus.battery = y.battery_status
                    end

                    # Optional env cache for sinks/telemetry.
                    if hasproperty(y, :rho_kgm3)
                        rho = getproperty(y, :rho_kgm3)
                        if isfinite(rho)
                            sim.bus.env = EnvSample(rho_kgm3 = rho, temp_k = sim.bus.env.temp_k)
                        end
                    end
                    if hasproperty(y, :temp_k)
                        temp = getproperty(y, :temp_k)
                        if isfinite(temp)
                            sim.bus.env = EnvSample(rho_kgm3 = sim.bus.env.rho_kgm3, temp_k = temp)
                        end
                    end
                else
                    sim.outputs.plant_y = nothing
                    sim.outputs.derived_valid = false
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
                # Autopilot consumes bus.est/battery and produces bus.cmd.
                update!(sim.autopilot, sim.bus, sim.plant, sim.t_us)
                if sim.cfg.mode == MODE_RECORD
                    record!(sim.recorder, :cmd, sim.t_us, sim.bus.cmd)
                end
            end

        elseif stage === :plant_discontinuities
            if ev.due_ap
                # Boundary-time plant discontinuities at autopilot ticks.
                if applicable(plant_on_autopilot_tick, sim.dynfun, sim.plant, sim.bus.cmd)
                    sim.plant = plant_on_autopilot_tick(sim.dynfun, sim.plant, sim.bus.cmd)
                end
            end

        elseif stage === :logging
            if ev.due_log
                # Boundary-time logs are *pre-step* snapshots.
                record!(sim.recorder, :plant, sim.t_us, sim.plant)

                # Always record battery on the log axis so tier0 recordings are schema-stable,
                # even for simplified dynamics models that do not implement plant_outputs(...).
                record!(sim.recorder, :battery, sim.t_us, sim.bus.battery)

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

@inline function _rb_state(x)
    if hasproperty(x, :rb)
        return getproperty(x, :rb)
    end
    error("Plant state does not expose an `rb` field; cannot emit rigid-body logs. Got: " * string(typeof(x)))
end

@inline function _rotor_omega_tuple(plant)::NTuple{4,Float64}
    if hasproperty(plant, :rotor_ω)
        ω = getproperty(plant, :rotor_ω)
        n = length(ω)
        return (
            n >= 1 ? ω[1] : NaN,
            n >= 2 ? ω[2] : NaN,
            n >= 3 ? ω[3] : NaN,
            n >= 4 ? ω[4] : NaN,
        )
    end
    return (NaN, NaN, NaN, NaN)
end

@inline function _rotor_thrust_tuple(y)::NTuple{4,Float64}
    if y === nothing || !hasproperty(y, :rotors)
        return (NaN, NaN, NaN, NaN)
    end
    rot = getproperty(y, :rotors)
    rot === nothing && return (NaN, NaN, NaN, NaN)
    th = rot.thrust_n
    n = length(th)
    return (
        n >= 1 ? th[1] : NaN,
        n >= 2 ? th[2] : NaN,
        n >= 3 ? th[3] : NaN,
        n >= 4 ? th[4] : NaN,
    )
end

@inline function _to_ntuple3_float(v)
    return (Float64(v[1]), Float64(v[2]), Float64(v[3]))
end

@inline function _autopilot_log_fields(ap)
    pos_sp = (NaN, NaN, NaN)
    vel_sp = (NaN, NaN, NaN)
    acc_sp = (NaN, NaN, NaN)
    yaw_sp = NaN
    yawspeed_sp = NaN
    nav_state = Int32(-1)
    arming_state = Int32(-1)
    mission_seq = Int32(0)
    mission_count = Int32(0)
    mission_finished = Int32(0)

    if hasproperty(ap, :last_out)
        out = getproperty(ap, :last_out)
        if out !== nothing
            if hasproperty(out, :trajectory_setpoint_position)
                pos_sp = _to_ntuple3_float(getproperty(out, :trajectory_setpoint_position))
            end
            if hasproperty(out, :trajectory_setpoint_velocity)
                vel_sp = _to_ntuple3_float(getproperty(out, :trajectory_setpoint_velocity))
            end
            if hasproperty(out, :trajectory_setpoint_acceleration)
                acc_sp = _to_ntuple3_float(getproperty(out, :trajectory_setpoint_acceleration))
            end
            if hasproperty(out, :trajectory_setpoint_yaw)
                yaw_sp = Float64(getproperty(out, :trajectory_setpoint_yaw))
            end
            if hasproperty(out, :trajectory_setpoint_yawspeed)
                yawspeed_sp = Float64(getproperty(out, :trajectory_setpoint_yawspeed))
            end
            if hasproperty(out, :nav_state)
                nav_state = Int32(getproperty(out, :nav_state))
            end
            if hasproperty(out, :arming_state)
                arming_state = Int32(getproperty(out, :arming_state))
            end
            if hasproperty(out, :mission_seq)
                mission_seq = Int32(getproperty(out, :mission_seq))
            end
            if hasproperty(out, :mission_count)
                mission_count = Int32(getproperty(out, :mission_count))
            end
            if hasproperty(out, :mission_finished)
                mission_finished = Int32(getproperty(out, :mission_finished))
            end
        end
    end

    return (
        pos_sp,
        vel_sp,
        acc_sp,
        yaw_sp,
        yawspeed_sp,
        nav_state,
        arming_state,
        mission_seq,
        mission_count,
        mission_finished,
    )
end

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

    pos_sp, vel_sp, acc_sp, yaw_sp, yawspeed_sp, nav_state, arming_state, mission_seq, mission_count, mission_finished =
        _autopilot_log_fields(sim.autopilot)

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
            battery = sim.bus.battery,
            rotor_omega = rotor_omega,
            rotor_thrust = rotor_thrust,
            pos_sp = pos_sp,
            vel_sp = vel_sp,
            acc_sp = acc_sp,
            yaw_sp = yaw_sp,
            yawspeed_sp = yawspeed_sp,
            nav_state = nav_state,
            arming_state = arming_state,
            mission_seq = mission_seq,
            mission_count = mission_count,
            mission_finished = mission_finished,
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
    t_us == sim.t_us || error("Engine/scheduler time mismatch: sched=$t_us engine=$(sim.t_us)")

    next_t_us = next_us(sim.sched)
    dt_us = next_t_us - t_us
    dt_s = Float64(dt_us) * 1e-6

    # Integrate the plant over [t, t+dt] with held inputs.
    u = PlantInput(cmd = sim.bus.cmd, wind_ned = sim.bus.wind_ned, faults = sim.bus.faults)

    reset!(sim.integrator)
    sim.plant = step_integrator(sim.integrator, sim.dynfun, sim.t_s, sim.plant, u, dt_s)

    # Optional post-step projection into physical bounds.
    if applicable(plant_project, sim.dynfun, sim.plant)
        sim.plant = plant_project(sim.dynfun, sim.plant)
    end

    # Advance scheduler to the new boundary.
    advance_evt!(sim.sched) || error("advance_evt! failed unexpectedly")

    # Advance authoritative time.
    sim.t_us = current_us(sim.sched)
    sim.t_s = Float64(sim.t_us) * 1e-6

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
