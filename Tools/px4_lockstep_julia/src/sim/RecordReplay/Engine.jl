"""Bus-driven hybrid simulation engine (Option A).

This engine is the execution core for record/replay:

- It advances time on the `Timeline.evt` axis.
- At each boundary time `t_us`, it processes all *due* discrete sources in a
  deterministic order.
- It integrates the continuous plant from `t_us` to the next boundary using the
  configured integrator and dynamics function.

This file is intentionally "top-down" with clear TODO seams. The initial milestone is
**plant-only replay**:

- Use `ReplayAutopilotSource` (publishes recorded commands)
- Use `ReplayWindSource` (publishes recorded wind)
- Integrate the plant open-loop (no PX4 involved)

Once that works, we layer in live PX4 and recording backends.
"""

using ..Plant: PlantInput, PlantState, sync_components_from_plant!
using ..Integrators: AbstractIntegrator, step_integrator, reset!, last_stats
import ..plant_outputs
using ..Vehicles: DirectActuators

# Optional compatibility: allow scenario helpers to interrogate legacy mutable
# components (propulsion motor enabled flags, battery SOC triggers, etc.) while
# the canonical truth lives in `PlantState`.
import ..PlantSimulation: PlantDynamicsWithContact

# Sources + recorder live in this module.

"""Record/replay mode for the engine."""
@enum EngineMode begin
    MODE_RECORD
    MODE_REPLAY
end

"""High-level engine configuration.

Notes
-----
- `dynfun` is the continuous RHS function with signature `f(t_s, x, u)`.
  For full-plant integration, this is typically `Sim.PlantDynamicsWithContact(...)`.
- `integrator` is one of `Sim.Integrators.*`.

TODO
----
- Add explicit "record tier" selection.
- Add runtime asserts that replay traces match the timeline axes.
"""
Base.@kwdef struct EngineConfig
    mode::EngineMode = MODE_REPLAY
end

"""Cursor for O(1) axis membership checks during the engine loop."""
mutable struct AxisCursor
    axis::TimeAxis
    idx::Int
end

AxisCursor(axis::TimeAxis) = AxisCursor(axis, 1)

@inline function reset_cursor!(cursor::AxisCursor)
    cursor.idx = 1
    return nothing
end

@inline function on_axis!(cursor::AxisCursor, t_us::UInt64)::Bool
    idx = cursor.idx
    if idx <= length(cursor.axis.t_us) && cursor.axis.t_us[idx] == t_us
        cursor.idx = idx + 1
        return true
    end
    return false
end

"""A bus-driven simulation engine instance.

Type parameters
---------------
- `PS`: plant state type (typically `PlantState{N}`)
- `D`: dynamics function type
- `I`: integrator type
- `AP/W/S/E`: source types
- `R`: recorder type
"""
mutable struct BusEngine{PS,D,I,AP,W,S,E,R}
    cfg::EngineConfig
    timeline::Timeline

    bus::SimBus
    plant::PS

    dynfun::D
    integrator::I

    autopilot::AP
    wind::W
    scenario::S
    estimator::E

    recorder::R

    ap_cursor::AxisCursor
    wind_cursor::AxisCursor
    log_cursor::AxisCursor
    scn_cursor::AxisCursor
end

@inline _post_interval_sync!(plant, dynfun) = nothing

"""Best-effort compatibility sync.

When the RHS functor is `PlantDynamicsWithContact`, it contains references to the
legacy mutable component objects (actuators/propulsion/battery). Some scenario
helpers and debug tools still look at those mutable objects.

The canonical truth during integration is `PlantState`, so after each integrated
interval we optionally synchronize those legacy objects from the plant state.

This is not required for deterministic plant replay, but it makes incremental
migration smoother.
"""
function _post_interval_sync!(plant, dynfun::PlantDynamicsWithContact)
    sync_components_from_plant!(
        plant,
        dynfun.motor_actuators,
        dynfun.servo_actuators,
        dynfun.propulsion,
        dynfun.battery,
    )
    return nothing
end

function BusEngine(
    cfg::EngineConfig,
    timeline::Timeline,
    bus::SimBus,
    plant::PS,
    dynfun::D,
    integrator::I,
    autopilot::AP,
    wind::W,
    scenario::S,
    estimator::E,
    recorder::R,
) where {PS,D,I,AP,W,S,E,R}
    return BusEngine(
        cfg,
        timeline,
        bus,
        plant,
        dynfun,
        integrator,
        autopilot,
        wind,
        scenario,
        estimator,
        recorder,
        AxisCursor(timeline.ap),
        AxisCursor(timeline.wind),
        AxisCursor(timeline.log),
        AxisCursor(timeline.scn),
    )
end

"""Snap direct actuators to the held command at a boundary.

This mirrors `PlantSimulation` behavior so `PlantState` stays consistent for
algebraic actuators.
"""
function _snap_direct_actuators!(sim::BusEngine)
    sim.plant isa PlantState || return nothing
    sim.dynfun isa PlantDynamicsWithContact || return nothing

    motors_y = sim.plant.motors_y
    motors_ydot = sim.plant.motors_ydot
    servos_y = sim.plant.servos_y
    servos_ydot = sim.plant.servos_ydot

    updated = false
    if sim.dynfun.motor_actuators isa DirectActuators
        motors_y = sim.bus.cmd.motors
        motors_ydot = zero(motors_ydot)
        updated = true
    end
    if sim.dynfun.servo_actuators isa DirectActuators
        servos_y = sim.bus.cmd.servos
        servos_ydot = zero(servos_ydot)
        updated = true
    end

    if updated
        T = typeof(sim.plant)
        sim.plant = T(
            rb = sim.plant.rb,
            motors_y = motors_y,
            motors_ydot = motors_ydot,
            servos_y = servos_y,
            servos_ydot = servos_ydot,
            rotor_ω = sim.plant.rotor_ω,
            batt_soc = sim.plant.batt_soc,
            batt_v1 = sim.plant.batt_v1,
        )
    end
    return nothing
end

"""Process all discrete events due at time `t_us`.

Deterministic ordering (important!):
1) scenario
2) wind
3) derived outputs (from plant state)
4) estimator
5) autopilot
6) logging/recording

TODO
----
- Add configurable ordering only if we have a compelling reason.
"""
function process_events_at!(sim::BusEngine, t_us::UInt64)
    sim.bus.time_us = t_us
    t_s = Float64(t_us) * 1e-6

    # 1) Scenario update.
    #
    # Rationale: even if the scenario's *discrete* events are scheduled on the
    # `timeline.scn` axis, scenarios often also produce *piecewise-constant*
    # outputs (e.g., arming/mode commands, landed overrides) that are consumed at
    # autopilot ticks. Calling the scenario on *every* boundary keeps these
    # signals up-to-date without requiring additional coupling.
    update!(sim.scenario, sim.bus, sim.plant, t_us)

    # Record scenario outputs.
    #
    # Design note: we record the *bus outputs* (ap_cmd / landed / faults) rather
    # than replaying opaque event payloads. This keeps record/replay stable across
    # scenario implementation changes.
    #
    # Important nuance for dynamic scenarios:
    # --------------------------------------
    # Scenarios may change faults based on state (e.g. `When(...)`). Those changes
    # can occur at *any* event boundary time, not just at static AtTime events.
    #
    # To avoid missing transitions in replay, we record the fault signal on the
    # **event axis** (`timeline.evt`) under the `*_evt` stream names.
    if sim.cfg.mode == MODE_RECORD
        # Streams aligned to `timeline.evt` (one sample per boundary).
        record!(sim.recorder, :ap_cmd_evt, t_us, sim.bus.ap_cmd)
        record!(sim.recorder, :landed_evt, t_us, sim.bus.landed)
        record!(sim.recorder, :faults_evt, t_us, sim.bus.faults)

        # Backward/diagnostic: also record on the scenario axis (small stream)
        # for static AtTime boundaries.
        if on_axis!(sim.scn_cursor, t_us)
            record!(sim.recorder, :ap_cmd, t_us, sim.bus.ap_cmd)
            record!(sim.recorder, :landed, t_us, sim.bus.landed)
            record!(sim.recorder, :faults, t_us, sim.bus.faults)
        end
    end

    # 2) Wind update
    if on_axis!(sim.wind_cursor, t_us)
        update!(sim.wind, sim.bus, sim.plant, t_us)
        if sim.cfg.mode == MODE_RECORD
            record!(sim.recorder, :wind_ned, t_us, sim.bus.wind_ned)
        end
    end

    # 2.5) Derived bus outputs from the current plant state.
    # This is especially important for PX4 injection (battery status) at boundary times.
    begin
        u = PlantInput(
            cmd = sim.bus.cmd,
            wind_ned = sim.bus.wind_ned,
            faults = sim.bus.faults,
        )
        if applicable(plant_outputs, sim.dynfun, t_s, sim.plant, u)
            outs = plant_outputs(sim.dynfun, t_s, sim.plant, u)
            if outs.battery_status !== nothing
                sim.bus.battery = outs.battery_status
            end
        end
    end

    # 3) Autopilot tick
    if on_axis!(sim.ap_cursor, t_us)
        # Estimator update should conceptually happen *before* autopilot, because
        # it produces the PX4-facing observation. In this first pass, estimator
        # is optional and often a no-op.
        update!(sim.estimator, sim.bus, sim.plant, t_us)

        update!(sim.autopilot, sim.bus, sim.plant, t_us)

        _snap_direct_actuators!(sim)

        # Record cmd stream (Tier-0)
        if sim.cfg.mode == MODE_RECORD
            record!(sim.recorder, :cmd, t_us, sim.bus.cmd)
        end
    end

    # 4) Logging / recording
    if on_axis!(sim.log_cursor, t_us)
        # Plant/battery snapshots are *always* useful, even in replay mode
        # (e.g., integrator sweeps comparing trajectories). `NullRecorder` makes
        # this effectively free if the caller doesn't want them.
        record!(sim.recorder, :plant, t_us, sim.plant)
        record!(sim.recorder, :battery, t_us, sim.bus.battery)
    end

    return nothing
end

"""Integrate plant from `t0_us` to `t1_us`.

Inputs are derived from the current bus state and held constant over the interval.
"""
function integrate_interval!(sim::BusEngine, t0_us::UInt64, t1_us::UInt64)
    t1_us >= t0_us || error("integrate_interval!: t1_us < t0_us")
    t1_us == t0_us && return nothing

    t0_s = Float64(t0_us) * 1e-6
    dt_s = Float64(t1_us - t0_us) * 1e-6

    # Inputs held constant over [t0, t1)
    u = PlantInput(cmd = sim.bus.cmd, wind_ned = sim.bus.wind_ned, faults = sim.bus.faults)

    # IMPORTANT: reset integrator history at the start of each interval.
    # This matches the current PlantSimulation semantics and avoids hidden state
    # (e.g., step size history) leaking across event boundaries.
    reset!(sim.integrator)

    sim.plant = step_integrator(sim.integrator, sim.dynfun, t0_s, sim.plant, u, dt_s)

    # Optional sync: keep any legacy mutable component objects coherent with the
    # canonical plant state (useful for scenarios / debugging).
    _post_interval_sync!(sim.plant, sim.dynfun)

    return nothing
end

"""Run the bus engine from `timeline.t0_us` to `timeline.t_end_us`.

TODO
----
- Provide hooks for progress reporting.
- Provide deterministic "stop at exactly t_end" asserts.
"""
function run!(sim::BusEngine)
    reset_cursor!(sim.ap_cursor)
    reset_cursor!(sim.wind_cursor)
    reset_cursor!(sim.log_cursor)
    reset_cursor!(sim.scn_cursor)

    evt = sim.timeline.evt.t_us
    n = length(evt)
    n >= 2 || error("timeline.evt must have at least 2 times")

    # Main loop: process events at t_k, integrate to t_{k+1}
    for k = 1:(n-1)
        t0_us = evt[k]
        t1_us = evt[k+1]

        process_events_at!(sim, t0_us)
        integrate_interval!(sim, t0_us, t1_us)
    end

    # Final boundary events
    process_events_at!(sim, evt[end])

    finalize!(sim.recorder)
    return nothing
end

"""Convenience constructor for a typical plant-only replay engine.

This constructor is intentionally narrow: it wires up a bus, a timeline and sources.

TODO
----
- Add a richer builder that can construct live sources from sim configs.
"""
function plant_replay_engine(;
    timeline::Timeline,
    plant0,
    dynfun,
    integrator::AbstractIntegrator,
    cmd_trace::ZOHTrace{ActuatorCommand},
    wind_trace::SampleHoldTrace{Vec3},
    scenario::AbstractScenarioSource = NullScenarioSource(),
    estimator::AbstractEstimatorSource = NullEstimatorSource(),
    recorder::AbstractRecorder = NullRecorder(),
)
    bus = SimBus(time_us = timeline.t0_us)

    ap = ReplayAutopilotSource(cmd_trace)
    wind = ReplayWindSource(wind_trace)

    # Scenario/estimator are optional. For integrator sweeps under plant-affecting
    # faults, pass a `ReplayScenarioSource` so bus.faults is replayed.
    scn = scenario
    est = estimator

    cfg = EngineConfig(mode = MODE_REPLAY)

    return BusEngine(
        cfg,
        timeline,
        bus,
        plant0,
        dynfun,
        integrator,
        ap,
        wind,
        scn,
        est,
        recorder,
    )
end

"""Convenience constructor for a typical *live* plant recording run.

This is just a thin helper around the `BusEngine` constructor to reduce boilerplate
in examples and user scripts.

Notes
-----
* The caller is responsible for providing *live* sources (e.g. `LiveAutopilotSource`).
* For large runs, you will likely want a streaming recorder (HDF5) rather than
  `InMemoryRecorder`.
"""
function plant_record_engine(;
    timeline::Timeline,
    plant0,
    dynfun,
    integrator::AbstractIntegrator,
    autopilot::AbstractAutopilotSource,
    wind::AbstractWindSource,
    scenario::AbstractScenarioSource = NullScenarioSource(),
    estimator::AbstractEstimatorSource = NullEstimatorSource(),
    recorder::AbstractRecorder = InMemoryRecorder(),
)
    bus = SimBus(time_us = timeline.t0_us)
    cfg = EngineConfig(mode = MODE_RECORD)
    return BusEngine(
        cfg,
        timeline,
        bus,
        plant0,
        dynfun,
        integrator,
        autopilot,
        wind,
        scenario,
        estimator,
        recorder,
    )
end
