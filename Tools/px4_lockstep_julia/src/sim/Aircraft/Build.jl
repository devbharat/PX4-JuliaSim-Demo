"""Spec → Engine builder.

This file is the implementation of the **aircraft composition layer**:

* It consumes an `AircraftSpec` (instances + connections + integration config)
* It produces the fully wired simulation inputs:
  - `timeline`
  - `plant0`
  - `dynfun`
  - `integrator`
  - runtime sources (autopilot/wind/scenario/estimator)

The builder applies spec-driven PX4 parameters for allocator geometry (CA_*),
enabling multi-rotor layouts to drive PX4 directly.
"""

using Random
using StaticArrays

using ..Runtime
using ..Recording
using ..Sources

using ..Plant
using ..Types
using ..RigidBody: RigidBodyState
using ..Vehicles
using ..Propulsion
using ..Powertrain
using ..PlantModels
using ..Environment
using ..Scenario
using ..Estimators

using ..Autopilots
using ..Integrators
using ..Contacts

using PX4Lockstep:
    param_set!, param_notify!, param_get, param_preinit_set!, control_alloc_update_params!

const _SIM = parentmodule(@__MODULE__)

include("build/Types.jl")
include("build/BuildTimeline.jl")
include("build/BuildPlant.jl")
include("build/BuildSources.jl")
include("build/BuildPX4.jl")

function _spec_summary(spec::AircraftSpec)::String
    nm = spec.name
    nmot = length(spec.actuation.motors)
    nb = length(spec.power.batteries)
    ns = length(spec.sensors)
    return "name=$(nm); motors=$(nmot); batteries=$(nb); sensors=$(ns)"
end


# -----------------
# Instance builder
# -----------------

"""Build a fully wired aircraft instance for a given run mode.

Returns a `BuildParts` containing the simulation inputs and any live PX4 handle.
Call `Autopilots.close!(inst.px4_handle)` when finished.
"""
function build_aircraft_instance(
    spec::AircraftSpec;
    mode::Symbol = :live,
    recording_in = nothing,
    telemetry = spec.telemetry,
    log_sinks = spec.log_sinks,
)
    # ---------
    # REPLAY
    # ---------
    if mode === :replay
        # Load recording if a path was provided.
        rec = if recording_in isa Recording.Tier0Recording
            recording_in
        elseif recording_in isa AbstractString
            Recording.read_recording(recording_in)
        else
            throw(
                ArgumentError(
                    "build_aircraft_instance(mode=:replay) requires recording_in (Tier0Recording or path)",
                ),
            )
        end

        traces = Recording.tier0_traces(rec)
        scn_tr = try
            Recording.scenario_traces(rec)
        catch e
            throw(
                ArgumentError(
                    "Recording is missing scenario streams needed for replay (faults/ap_cmd/landed). " *
                    "Re-record with record_faults_evt=true.\n\nOriginal error: $e",
                ),
            )
        end

        # Environment: disable live wind (wind comes from trace).
        env = _build_env_replay(spec)

        # Build param objects from spec but override initial RB to match the recording.
        vehicle = _build_vehicle(spec; x0_override = rec.plant0.rb)

        # Multi-battery power network (recording must match spec topology).
        batteries = _build_batteries(spec)
        power_net = _build_power_network(spec)

        # Explicit actuator -> physical propulsor mapping.
        N = length(spec.actuation.motors)
        if length(rec.plant0.rotor_ω) != N
            throw(
                ArgumentError(
                    "Replay recording motor count does not match spec: recording N=$(length(rec.plant0.rotor_ω)) spec N=$(N)",
                ),
            )
        end
        if length(rec.plant0.power.soc) != length(batteries)
            throw(
                ArgumentError(
                    "Replay recording battery count does not match spec: recording B=$(length(rec.plant0.power.soc)) spec B=$(length(batteries))",
                ),
            )
        end
        motor_map = Vehicles.MotorMap{N}(
            SVector{N,Int}(ntuple(i -> spec.actuation.motors[i].channel, N)),
        )
        M = length(spec.actuation.servos)
        servo_map = Vehicles.ServoMap{M}(
            SVector{M,Int}(ntuple(i -> spec.actuation.servos[i].channel, M)),
        )

        dynfun = PlantModels.CoupledMultirotorModel(
            vehicle.model,
            env,
            spec.plant.contact,
            vehicle.motor_actuators,
            vehicle.servo_actuators,
            vehicle.propulsion,
            batteries,
            power_net;
            motor_map = motor_map,
            servo_map = servo_map,
        )

        integrator = _resolve_integrator(spec)

        sources = _build_replay_sources(traces, scn_tr)

        meta = Dict{Symbol,Any}(:aircraft_spec_summary => _spec_summary(spec))

        return BuildParts(
            timeline = rec.timeline,
            plant0 = rec.plant0,
            dynfun = dynfun,
            integrator = integrator,
            sources = sources,
            meta = meta,
            run_mode = mode,
            telemetry = telemetry,
            log_sinks = log_sinks,
            px4_handle = nothing,
        )
    end

    # ---------
    # LIVE / RECORD
    # ---------

    # Environment with live wind.
    env = _build_env_live(spec)

    # Scenario + timeline.
    scenario_src = _build_live_scenario_source(spec)
    timeline = _build_default_timeline(
        t_end_s = spec.timeline.t_end_s,
        dt_autopilot_s = spec.timeline.dt_autopilot_s,
        dt_wind_s = spec.timeline.dt_wind_s,
        dt_log_s = spec.timeline.dt_log_s,
        dt_phys_s = spec.timeline.dt_phys_s,
        scenario_source = scenario_src,
    )

    # Plant-side param objects.
    vehicle = _build_vehicle(spec)

    # Multi-battery power network.
    batteries = _build_batteries(spec)
    power_net = _build_power_network(spec)

    # Explicit actuator -> physical propulsor mapping.
    N = length(spec.actuation.motors)
    motor_map = Vehicles.MotorMap{N}(
        SVector{N,Int}(ntuple(i -> spec.actuation.motors[i].channel, N)),
    )
    M = length(spec.actuation.servos)
    servo_map = Vehicles.ServoMap{M}(
        SVector{M,Int}(ntuple(i -> spec.actuation.servos[i].channel, M)),
    )

    dynfun = PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        spec.plant.contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        batteries,
        power_net;
        motor_map = motor_map,
        servo_map = servo_map,
    )

    integrator = _resolve_integrator(spec)

    plant0 = Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        batteries,
    )

    # PX4 autopilot (lockstep) init.
    ap = _build_px4_autopilot(spec, vehicle, plant0)

    sources = _build_live_sources(spec, env, scenario_src, ap)

    meta = Dict{Symbol,Any}(
        :home => spec.home,
        :mission => spec.px4.mission_path,
        :aircraft_spec_summary => _spec_summary(spec),
    )

    return BuildParts(
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integrator,
        sources = sources,
        meta = meta,
        run_mode = mode,
        telemetry = telemetry,
        log_sinks = log_sinks,
        px4_handle = ap,
    )
end

include("build/BuildEngine.jl")
