"""Spec → Engine builder (Phase 3).

This file is the implementation of the **aircraft composition layer**:

* It consumes an `AircraftSpec` (instances + connections + integration config)
* It produces the fully wired simulation inputs:
  - `timeline`
  - `plant0`
  - `dynfun`
  - `integrator`
  - runtime sources (autopilot/wind/scenario/estimator)

Phase 3 extends the Phase 1 instance model with PX4 parameter injection for
allocator geometry (CA_*), enabling multi-rotor layouts to drive PX4 directly.
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

using ..Autopilots
using ..Integrators
using ..Contacts

using PX4Lockstep:
    param_set!,
    param_notify!,
    param_get,
    param_preinit_set!,
    control_alloc_update_params!


"""Internal build output.

This is intentionally *not* exported yet. It is the structured result of
`build_aircraft_instance(...)` and is consumed by `build_engine(...)`.
"""
struct AircraftInstance{TL,PS,D,INT,S}
    timeline::TL
    plant0::PS
    dynfun::D
    integrator::INT
    sources::S
    meta::Dict{Symbol,Any}
end


# -----------------
# Helper builders
# -----------------

@inline function _resolve_integrator(spec::AircraftSpec)
    integ = spec.plant.integrator
    if integ isa Symbol
        # Reuse the existing integrator factory for now (Iris workflow helper).
        return _SIM.iris_integrator(integ)
    end
    return integ
end

function _build_actuator_model(mspec::AbstractActuatorModelSpec, N::Int)
    if mspec isa DirectActuatorSpec
        return Vehicles.DirectActuators()
    elseif mspec isa FirstOrderActuatorSpec
        return Vehicles.FirstOrderActuators{N}(τ = mspec.τ)
    elseif mspec isa SecondOrderActuatorSpec
        return Vehicles.SecondOrderActuators{N}(
            ωn = mspec.ωn,
            ζ = mspec.ζ,
            rate_limit = mspec.rate_limit,
        )
    else
        error("Unknown actuator model spec: $(typeof(mspec))")
    end
end

function _select_battery_spec(spec::AircraftSpec, id::BatteryId)
    for b in spec.power.batteries
        b.id == id && return b
    end
    error("Battery id=$(id) not found in spec.power.batteries")
end

function _build_battery(bs::BatterySpec)
    if bs.model === :thevenin
        return Powertrain.TheveninBattery(
            capacity_ah = bs.capacity_ah,
            soc0 = bs.soc0,
            ocv_soc = bs.ocv_soc,
            ocv_v = bs.ocv_v,
            r0 = bs.r0,
            r1 = bs.r1,
            c1 = bs.c1,
            v1_0 = bs.v1_0,
            min_voltage_v = bs.min_voltage_v,
            low_thr = bs.low_thr,
            crit_thr = bs.crit_thr,
            emerg_thr = bs.emerg_thr,
        )
    elseif bs.model === :ideal
        # Useful for tests and some prototyping.
        V = isempty(bs.ocv_v) ? 12.0 : bs.ocv_v[end]
        return Powertrain.IdealBattery(
            capacity_ah = bs.capacity_ah,
            soc0 = bs.soc0,
            voltage_v = V,
            low_thr = bs.low_thr,
            crit_thr = bs.crit_thr,
            emerg_thr = bs.emerg_thr,
        )
    else
        error("Unsupported battery model=$(bs.model) (expected :thevenin|:ideal)")
    end
end

function _build_vehicle(spec::AircraftSpec; x0_override::Union{Nothing,RigidBodyState} = nothing)
    a = spec.airframe

    # Phase 2 supports a generic multirotor rigid-body model with arbitrary propulsor count.
    a.kind in (:iris_quadrotor, :multirotor) ||
        error("Unsupported airframe.kind=$(a.kind) (Phase 2 supports :iris_quadrotor|:multirotor)")

    # Motor/servo actuator models are sized to match the PX4 ABI arrays.
    motor_act = _build_actuator_model(spec.actuation.motor_actuators, 12)
    servo_act = _build_actuator_model(spec.actuation.servo_actuators, 8)

    # Rigid-body model params.
    N = length(spec.actuation.motors)
    length(a.rotor_pos_body_m) == N ||
        error("airframe.rotor_pos_body_m length=$(length(a.rotor_pos_body_m)) must match actuation.motors length=$(N)")
    rotor_pos = SVector{N,Vec3}(ntuple(i -> a.rotor_pos_body_m[i], N))

    # Propulsor axes (Phase 4): optional spec field, normalized here.
    axis_src = a.rotor_axis_body_m
    length(axis_src) == N ||
        error("airframe.rotor_axis_body_m length=$(length(axis_src)) must match actuation.motors length=$(N)")
    rotor_axis = SVector{N,Vec3}(
        ntuple(
            i -> begin
                v = axis_src[i]
                n2 = v[1] * v[1] + v[2] * v[2] + v[3] * v[3]
                n2 > 1e-12 || error("airframe.rotor_axis_body_m[$i] is near-zero: $v")
                invn = inv(sqrt(n2))
                v .* invn
            end,
            N,
        ),
    )
    params = Vehicles.QuadrotorParams{N}(
        mass = a.mass_kg,
        inertia_diag = a.inertia_diag_kgm2,
        rotor_pos_body = rotor_pos,
        rotor_axis_body = rotor_axis,
        linear_drag = a.linear_drag,
        angular_damping = a.angular_damping,
    )

    model = if a.kind === :iris_quadrotor
        Vehicles.IrisQuadrotor(params = params)
    else
        Vehicles.GenericMultirotor{N}(params)
    end

    # Propulsion (Iris-like default motor+prop set).
    hover_T = a.mass_kg * 9.80665 / Float64(N)
    p = a.propulsion
    prop = Propulsion.default_iris_quadrotor_set(
        N = N,
        km_m = p.km_m,
        V_nom = p.V_nom,
        ρ_nom = p.rho_nom,
        thrust_hover_per_rotor_n = hover_T,
        rotor_radius_m = p.rotor_radius_m,
        inflow_kT = p.inflow_kT,
        inflow_kQ = p.inflow_kQ,
    )
    if p.rotor_dir !== nothing
        length(p.rotor_dir) == N ||
            error("propulsion.rotor_dir length=$(length(p.rotor_dir)) does not match N=$(N)")
        prop.rotor_dir = SVector{N,Float64}(ntuple(i -> Float64(p.rotor_dir[i]), N))
    end

    x0 = x0_override === nothing ? a.x0 : x0_override
    return Vehicles.VehicleInstance(model, motor_act, servo_act, prop, x0)
end



# -----------------
# PX4 parameterization helpers (Phase 3)
# -----------------

"""Derive PX4 control allocator (CA_*) parameters from the aircraft spec.

This matches the old lockstep C-side Iris defaults, but is now spec-driven.
"""
function _derive_ca_params(spec::AircraftSpec, vehicle::Vehicles.VehicleInstance)
    cfg = spec.px4.lockstep_config
    cfg.enable_control_allocator == 0 && return PX4ParamSpec[]

    N = length(spec.actuation.motors)
    pos = spec.airframe.rotor_pos_body_m
    length(pos) == N || error("Cannot derive CA params: rotor_pos_body_m length != motor count")

    # Optional rotor/propulsor axes (Phase 4). Defaults to classic multirotor axis_b=(0,0,1)
    # so the thrust vector (force) points along -Z.
    axis_src = spec.airframe.rotor_axis_body_m
    length(axis_src) == N || error("Cannot derive CA params: rotor_axis_body_m length != motor count")

    km_mag = spec.airframe.propulsion.km_m
    rotor_dir = vehicle.propulsion.rotor_dir

    params = PX4ParamSpec[
        PX4ParamSpec("CA_AIRFRAME", 0),              # multicopter (custom geometry)
        PX4ParamSpec("CA_ROTOR_COUNT", N),
    ]

    for i in 1:N
        p = pos[i]
        a = axis_src[i]
        n2 = a[1] * a[1] + a[2] * a[2] + a[3] * a[3]
        n2 > 1e-12 || error("Cannot derive CA params: rotor_axis_body_m[$i] is near-zero: $a")
        invn = inv(sqrt(n2))
        axis_b = a .* invn
        # PX4 CA_ROTOR*_A* expects the *thrust vector direction* in body frame.
        # Our convention stores propulsor axis such that F = -T * axis_b.
        axis_thrust = -axis_b
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PX", Float32(p[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PY", Float32(p[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_PZ", Float32(p[3])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_KM", Float32(km_mag * rotor_dir[i])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AX", Float32(axis_thrust[1])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AY", Float32(axis_thrust[2])))
        push!(params, PX4ParamSpec("CA_ROTOR$(i-1)_AZ", Float32(axis_thrust[3])))
    end

    return params
end

"""Apply a list of PX4 parameters to a running lockstep autopilot."""
function _apply_px4_params!(ap::Autopilots.PX4LockstepAutopilot, params::Vector{PX4ParamSpec})
    for p in params
        param_set!(ap.handle, p.name, p.value)
    end
    return nothing
end

function _debug_px4_params!(ap::Autopilots.PX4LockstepAutopilot, params::Vector{PX4ParamSpec})
    flag = lowercase(String(get(ENV, "PX4_LOCKSTEP_DEBUG_PARAMS", "")))
    if !(flag in ("1", "true", "yes", "on"))
        return nothing
    end

    if isempty(params)
        println("[PX4 params] (no spec params applied)")
        return nothing
    end

    println("[PX4 params] applied:")
    for p in params
        value = try
            param_get(ap.handle, p.name)
        catch err
            "<error: $(sprint(showerror, err))>"
        end
        println("  ", p.name, " = ", value)
    end
    return nothing
end


@inline function _is_ca_axis_param(name::AbstractString)::Bool
    # PX4 CA rotor axis params: CA_ROTOR<i>_AX / _AY / _AZ
    return startswith(name, "CA_ROTOR") && (endswith(name, "_AX") || endswith(name, "_AY") || endswith(name, "_AZ"))
end

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

Returns `(instance, ap)` where `ap` is the initialized PX4 autopilot handle for
live/record modes (and `nothing` for replay). Call `Autopilots.close!(ap)` when
finished.
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
            error("build_aircraft_instance(mode=:replay) requires recording_in (Tier0Recording or path)")
        end

        traces = Recording.tier0_traces(rec)
        scn_tr = try
            Recording.scenario_traces(rec)
        catch e
            error(
                "Recording is missing scenario streams needed for replay (faults/ap_cmd/landed). " *
                "Re-record with record_faults_evt=true.\n\nOriginal error: $e",
            )
        end

        # Environment: disable live wind (wind comes from trace).
        env = _SIM.iris_default_env_replay(home = spec.home)

        # Build param objects from spec but override initial RB to match the recording.
        vehicle = _build_vehicle(spec; x0_override = rec.plant0.rb)
        bs = _select_battery_spec(spec, spec.power.primary_battery)
        battery = _build_battery(bs)

        # Explicit actuator -> physical propulsor mapping (Phase 2).
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
            battery;
            motor_map = motor_map,
            servo_map = servo_map,
        )

        integrator = _resolve_integrator(spec)

        wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing
        scenario = Sources.ReplayScenarioSource(
            scn_tr.ap_cmd,
            scn_tr.landed,
            scn_tr.faults;
            wind_dist = wind_dist,
        )

        sources = (
            autopilot = Sources.ReplayAutopilotSource(traces.cmd),
            wind = Sources.ReplayWindSource(traces.wind_ned),
            scenario = scenario,
            estimator = Sources.NullEstimatorSource(),
        )

        meta = Dict{Symbol,Any}(
            :aircraft_spec_summary => _spec_summary(spec),
        )

        inst = AircraftInstance(
            rec.timeline,
            rec.plant0,
            dynfun,
            integrator,
            sources,
            meta,
        )
        return inst, nothing
    end

    # ---------
    # LIVE / RECORD
    # ---------

    # Environment with live wind.
    env = _SIM.iris_default_env_live(home = spec.home)

    # Scenario + timeline.
    scenario_obj = _SIM.iris_default_scenario()
    scenario_src = Sources.LiveScenarioSource(scenario_obj)
    timeline = _SIM.iris_timeline(
        t_end_s = spec.timeline.t_end_s,
        dt_autopilot_s = spec.timeline.dt_autopilot_s,
        dt_wind_s = spec.timeline.dt_wind_s,
        dt_log_s = spec.timeline.dt_log_s,
        dt_phys_s = spec.timeline.dt_phys_s,
        scenario_source = scenario_src,
    )

    # Wind + estimator.
    wind_src = Sources.LiveWindSource(env.wind, Random.Xoshiro(spec.seed), spec.timeline.dt_wind_s)
    estimator_obj = _SIM.iris_default_estimator(spec.timeline.dt_autopilot_s)
    estimator_src = Sources.LiveEstimatorSource(
        estimator_obj,
        Random.Xoshiro(spec.seed + 1),
        spec.timeline.dt_autopilot_s,
    )

    # Plant-side param objects.
    vehicle = _build_vehicle(spec)
    bs = _select_battery_spec(spec, spec.power.primary_battery)
    battery = _build_battery(bs)

    # Explicit actuator -> physical propulsor mapping (Phase 2).
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
        battery;
        motor_map = motor_map,
        servo_map = servo_map,
    )

    integrator = _resolve_integrator(spec)

    plant0 = Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    # Apply spec-driven PX4 parameters (Phase 3).
    px4_params = PX4ParamSpec[]
    if spec.px4.derive_ca_params
        append!(px4_params, _derive_ca_params(spec, vehicle))
    end
    # Explicit overrides win.
    append!(px4_params, spec.px4.params)

    # Some PX4 branches do not expose CA_ROTOR*_A* parameters (tilt/axis support). We
    # therefore stage those as *optional* and apply them after PX4 init if they exist.
    axis_params = PX4ParamSpec[]
    for p in px4_params
        if _is_ca_axis_param(p.name)
            push!(axis_params, p)
        else
            param_preinit_set!(p.name, p.value)
        end
    end

    # PX4 autopilot (lockstep) init.
    ap = Autopilots.init!(
        config = spec.px4.lockstep_config,
        libpath = spec.px4.libpath,
        home = spec.home,
        uorb_cfg = spec.px4.uorb_cfg,
        edge_trigger = spec.px4.edge_trigger,
    )

    # Apply optional CA_ROTOR*_A* params if the PX4 build supports them (Phase 4.3).
    axis_applied = false
    if !isempty(axis_params)
        for p in axis_params
            # param_get throws if param doesn't exist; treat that as "not supported".
            supported = try
                _ = param_get(ap.handle, p.name)
                true
            catch
                false
            end
            if supported
                param_set!(ap.handle, p.name, p.value)
                axis_applied = true
            end
        end
        # Reload CA params without a global param_notify.
        (axis_applied && spec.px4.lockstep_config.enable_control_allocator != 0) &&
            control_alloc_update_params!(ap.handle)
    end

    # Only broadcast a parameter update if explicit overrides were provided.
    isempty(spec.px4.params) || param_notify!(ap.handle)
    _debug_px4_params!(ap, px4_params)

    # Initial step at t=0 so PX4 has a consistent state baseline.
    _ = Autopilots.autopilot_step(
        ap,
        UInt64(0),
        plant0.rb.pos_ned,
        plant0.rb.vel_ned,
        plant0.rb.q_bn,
        plant0.rb.ω_body,
        Autopilots.AutopilotCommand();
        landed = true,
        battery = Powertrain.BatteryStatus(),
    )

    # Load mission if provided.
    if spec.px4.mission_path !== nothing
        Autopilots.load_mission!(ap, spec.px4.mission_path)
    end

    autopilot_src = Sources.LiveAutopilotSource(ap)

    sources = (
        autopilot = autopilot_src,
        wind = wind_src,
        scenario = scenario_src,
        estimator = estimator_src,
    )

    meta = Dict{Symbol,Any}(
        :home => spec.home,
        :mission => spec.px4.mission_path,
        :aircraft_spec_summary => _spec_summary(spec),
    )

    inst = AircraftInstance(
        timeline,
        plant0,
        dynfun,
        integrator,
        sources,
        meta,
    )
    return inst, ap
end


# -----------------
# Public entrypoint
# -----------------

"""Build and run an engine from an `AircraftSpec`.

This is the stable build entrypoint used by `Workflows.simulate_iris_mission`.

Modes
-----
* `:live`   -> return `Runtime.Engine`
* `:record` -> return `Recording.Tier0Recording` (and optionally write to `recording_out`)
* `:replay` -> return `Runtime.Engine`
"""
function build_engine(
    spec::AircraftSpec;
    mode::Symbol = :live,
    recording_in = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
    telemetry = spec.telemetry,
    log_sinks = spec.log_sinks,
)
    validate_spec(spec; mode = mode, recording_in = recording_in)

    # Phase 2 supports Iris preset and generic multirotor specs. Additional airframe
    # kinds will be enabled in later phases.

    inst, ap = build_aircraft_instance(
        spec;
        mode = mode,
        recording_in = recording_in,
        telemetry = telemetry,
        log_sinks = log_sinks,
    )

    try
        if mode === :record
            recorder = Recording.InMemoryRecorder()
            eng = _SIM.simulate(
                mode = :record,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                recorder = recorder,
                log_sinks = log_sinks,
            )

            rec = Recording.Tier0Recording(
                timeline = inst.timeline,
                plant0 = inst.plant0,
                recorder = recorder,
                meta = inst.meta,
            )
            if recording_out !== nothing
                Recording.write_recording(recording_out, rec)
            end
            return rec

        elseif mode === :replay
            eng = _SIM.simulate(
                mode = :replay,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            return eng

        elseif mode === :live
            eng = _SIM.simulate(
                mode = :live,
                timeline = inst.timeline,
                plant0 = inst.plant0,
                dynfun = inst.dynfun,
                integrator = inst.integrator,
                autopilot = inst.sources.autopilot,
                wind = inst.sources.wind,
                scenario = inst.sources.scenario,
                estimator = inst.sources.estimator,
                telemetry = telemetry,
                log_sinks = log_sinks,
            )
            return eng

        else
            error("build_engine: unknown mode=$(mode) (expected :live|:record|:replay)")
        end
    finally
        ap === nothing || Autopilots.close!(ap)
    end
end
