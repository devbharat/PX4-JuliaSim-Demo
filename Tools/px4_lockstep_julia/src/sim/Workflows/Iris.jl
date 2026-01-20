"""Iris workflow helpers.

Goal
----
Provide a turnkey Iris workflow on top of the canonical engine.

These are convenience wrappers around the building blocks in:
- `Sim.Runtime` (engine + timeline)
- `Sim.Sources` (live/replay sources)
- `Sim.Recording` (Tier0 recording)
- `Sim.PlantModels` (full-plant RHS)

They provide opinionated defaults for the common PX4 SITL use case: an Iris
quadrotor flying a waypoint mission.

For other vehicles or scenarios, use `Sim.simulate(...)` directly.
"""

using Random
using StaticArrays

import ..LockstepConfig

const IRIS_DEFAULT_HOME =
    Autopilots.HomeLocation(lat_deg = 47.397742, lon_deg = 8.545594, alt_msl_m = 488.0)

"""Return a default home location (used for local NED origin)."""
iris_default_home() = IRIS_DEFAULT_HOME

"""Build a default environment for a live PX4 run.

The wind model is intentionally nonzero by default so record/replay covers
wind sampling determinism.
"""
function iris_default_env_live(; home = iris_default_home())
    wind = Environment.OUWind(
        mean = Types.vec3(0.0, 0.0, 0.0),
        σ = Types.vec3(1.5, 1.5, 0.5),
        τ_s = 3.0,
    )
    atm = Environment.ISA1976()
    g = Environment.UniformGravity(9.80665)
    return Environment.EnvironmentModel(
        atmosphere = atm,
        wind = wind,
        gravity = g,
        origin = home,
    )
end

"""Build a default environment for replay.

During replay, wind is supplied via a recorded trace (`bus.wind_ned`), so the
environment wind model is set to `NoWind`.
"""
function iris_default_env_replay(; home = iris_default_home())
    atm = Environment.ISA1976()
    g = Environment.UniformGravity(9.80665)
    return Environment.EnvironmentModel(
        atmosphere = atm,
        wind = Environment.NoWind(),
        gravity = g,
        origin = home,
    )
end

"""Build a default Iris vehicle instance.

Notes
-----
- Motor actuators are `DirectActuators` by default because the propulsion model already
  contains first-order-ish electrical + rotor dynamics.
- Servo actuators are direct (Iris has no control surfaces).
"""
function iris_default_vehicle(; x0::RigidBody.RigidBodyState = RigidBody.RigidBodyState())
    model = Vehicles.IrisQuadrotor()

    motor_act = Vehicles.DirectActuators()
    servo_act = Vehicles.DirectActuators()

    # Calibrate km_m such that the electrical model matches a hover-ish operating point.
    hover_T = Vehicles.mass(model) * 9.80665 / 4.0
    propulsion = Propulsion.default_iris_quadrotor_set(
        km_m = 0.05,
        thrust_hover_per_rotor_n = hover_T,
    )

    return Vehicles.VehicleInstance(model, motor_act, servo_act, propulsion, x0)
end

"""Build a default Thevenin battery model for Iris."""
function iris_default_battery()
    # 3S LiPo-style defaults. These are intended as a reasonable closed-loop SITL
    # battery model, not a calibrated pack representation.
    return Powertrain.TheveninBattery(
        capacity_ah = 5.0,
        soc0 = 1.0,
        # simple OCV curve: 3.6-4.2V per cell -> 10.8-12.6V pack
        ocv_soc = [0.0, 1.0],
        ocv_v = [10.8, 12.6],
        r0 = 0.020,
        r1 = 0.010,
        c1 = 2000.0,
        v1_0 = 0.0,
        min_voltage_v = 9.9,
    )
end


"""Build a default contact model for Iris workflows.

Default is flat-ground penalty contact so missions that start on the ground do not
free-fall before takeoff. Use `Contacts.NoContact()` for pure flight.
"""
function iris_default_contact()
    return Contacts.FlatGroundContact()
end

"""Build a default mission scenario (arm + start mission)."""
function iris_default_scenario(; arm_time_s::Float64 = 1.0, mission_time_s::Float64 = 2.0)
    s = Scenario.EventScenario()
    Scenario.arm_at!(s, arm_time_s)
    Scenario.mission_start_at!(s, mission_time_s)
    return s
end

"""Default PX4 lockstep config for Iris workflows.

Disable commander until PX4 provides lockstep support, mirroring the legacy workflow.
"""
function iris_default_lockstep_config()
    return LockstepConfig(
        dataman_use_ram = 1,
        enable_commander = 0,
        enable_control_allocator = 1,
    )
end

"""Build a default estimator (truth + injected noise/bias + delay)."""
function iris_default_estimator(dt_ap_s::Float64)
    base = Estimators.NoisyEstimator(
        pos_sigma_m = Types.vec3(0.02, 0.02, 0.02),
        vel_sigma_mps = Types.vec3(0.05, 0.05, 0.05),
        yaw_sigma_rad = 0.01,
        rate_sigma_rad_s = Types.vec3(0.005, 0.005, 0.005),
        bias_tau_s = 50.0,
        rate_bias_sigma_rad_s = Types.vec3(0.001, 0.001, 0.001),
    )
    # Delay is step-quantized; ensure it is a multiple of dt_ap_s.
    return Estimators.DelayedEstimator(base; delay_s = 2 * dt_ap_s, dt_est = dt_ap_s)
end

"""Build the coupled multirotor plant model used by the canonical engine."""
function iris_dynfun(
    env,
    vehicle::Vehicles.VehicleInstance,
    battery;
    contact = Contacts.NoContact(),
)
    return PlantModels.CoupledMultirotorModel(
        vehicle.model,
        env,
        contact,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )
end

"""A tight RK45 integrator configuration suitable as a reference for comparisons."""
function iris_reference_integrator()
    return Integrators.RK45Integrator(
        rtol_pos = 1e-7,
        atol_pos = 1e-7,
        rtol_vel = 1e-7,
        atol_vel = 1e-7,
        rtol_ω = 1e-7,
        atol_ω = 1e-7,
        atol_att_rad = 1e-7,
        h_min = 1e-6,
        h_max = 0.02,
        plant_error_control = true,
        atol_rotor = 1e-2,
        atol_soc = 1e-6,
        atol_v1 = 1e-3,
    )
end

"""Pick a named integrator for Iris workflows.

Supported names: `:Euler`, `:RK4`, `:RK23`, `:RK45`.
"""
function iris_integrator(name::Symbol)
    if name === :Euler
        return Integrators.EulerIntegrator()
    elseif name === :RK4
        return Integrators.RK4Integrator()
    elseif name === :RK23
        return Integrators.RK23Integrator()
    elseif name === :RK45
        return Integrators.RK45Integrator()
    else
        error("Unknown integrator name=$name (expected :Euler|:RK4|:RK23|:RK45)")
    end
end

"""Create a canonical timeline for Iris workflows."""
function iris_timeline(;
    t_end_s::Float64,
    dt_autopilot_s::Float64,
    dt_wind_s::Float64,
    dt_log_s::Float64,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    scenario_source = nothing,
)
    t0_us = UInt64(0)
    t_end_us = Runtime.dt_to_us(t_end_s)
    dt_ap_us = Runtime.dt_to_us(dt_autopilot_s)
    dt_wind_us = Runtime.dt_to_us(dt_wind_s)
    dt_log_us = Runtime.dt_to_us(dt_log_s)
    dt_phys_us = dt_phys_s === nothing ? nothing : Runtime.dt_to_us(dt_phys_s)

    return Runtime.build_timeline_for_run(
        t0_us,
        t_end_us;
        dt_ap_us = dt_ap_us,
        dt_wind_us = dt_wind_us,
        dt_log_us = dt_log_us,
        dt_phys_us = dt_phys_us,
        scenario = scenario_source,
    )
end

"""Run an Iris mission with a clean, top-level UX.

Modes
-----
- `mode=:live`   : step PX4 live (no recording unless a recorder is provided)
- `mode=:record` : live PX4 + in-memory recorder, returns a `Tier0Recording`
- `mode=:replay` : replay from a Tier0Recording (or path) with deterministic sources

This is intended as the go-to user workflow.
"""
function simulate_iris_mission(;
    mode::Symbol = :live,
    mission_path::Union{Nothing,AbstractString} = get(ENV, "PX4_LOCKSTEP_MISSION", nothing),
    libpath::Union{Nothing,AbstractString} = get(ENV, "PX4_LOCKSTEP_LIB", nothing),
    lockstep_config = iris_default_lockstep_config(),
    recording_in::Union{Nothing,AbstractString} = nothing,
    recording_out::Union{Nothing,AbstractString} = nothing,
    t_end_s::Float64 = parse(Float64, get(ENV, "IRIS_T_END_S", "20.0")),
    dt_autopilot_s::Float64 = 0.004,
    dt_wind_s::Float64 = 0.001,
    dt_log_s::Float64 = 0.01,
    dt_phys_s::Union{Nothing,Float64} = nothing,
    seed::Integer = 1,
    integrator::Union{Symbol,Integrators.AbstractIntegrator} = :RK45,
    home = iris_default_home(),
    contact = iris_default_contact(),
    telemetry = Runtime.NullTelemetry(),
    log_sinks = nothing,
)
    integ = integrator isa Symbol ? iris_integrator(integrator) : integrator

    if mode === :replay
        recording_in === nothing &&
            error("simulate_iris_mission(mode=:replay) requires recording_in")
        rec = Recording.read_recording(recording_in)

        # Replay sources
        traces = Recording.tier0_traces(rec)
        scn_tr = try
            Recording.scenario_traces(rec)
        catch e
            error(
                "Recording is missing scenario streams needed for replay (faults/ap_cmd/landed). " *
                "Re-record with record_faults_evt=true.\n\nOriginal error: $e",
            )
        end

        wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing
        scenario = Sources.ReplayScenarioSource(
            scn_tr.ap_cmd,
            scn_tr.landed,
            scn_tr.faults;
            wind_dist = wind_dist,
        )
        wind = Sources.ReplayWindSource(traces.wind_ned)
        autopilot = Sources.ReplayAutopilotSource(traces.cmd)

        env = iris_default_env_replay(home = home)
        vehicle = iris_default_vehicle(; x0 = rec.plant0.rb)  # keep params; state comes from recording
        battery = iris_default_battery()
        dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

        # Plant initial state comes from recording.
        plant0 = rec.plant0

        eng = simulate(
            mode = :replay,
            timeline = rec.timeline,
            plant0 = plant0,
            dynfun = dynfun,
            integrator = integ,
            autopilot = autopilot,
            wind = wind,
            scenario = scenario,
            estimator = Sources.NullEstimatorSource(),
            telemetry = telemetry,
            recorder = Recording.NullRecorder(),
            log_sinks = log_sinks,
        )
        return eng
    end

    # Live or record uses PX4.
    mission_path === nothing &&
        error("No mission file provided (set mission_path or PX4_LOCKSTEP_MISSION)")

    # Build environment, vehicle, components.
    env = iris_default_env_live(home = home)
    vehicle = iris_default_vehicle()
    battery = iris_default_battery()
    scenario_obj = iris_default_scenario()
    estimator_obj = iris_default_estimator(dt_autopilot_s)
    dynfun = iris_dynfun(env, vehicle, battery; contact = contact)

    scenario_src = Sources.LiveScenarioSource(scenario_obj)
    wind_src = Sources.LiveWindSource(env.wind, Random.Xoshiro(seed), dt_wind_s)
    est_src =
        Sources.LiveEstimatorSource(estimator_obj, Random.Xoshiro(seed + 1), dt_autopilot_s)

    # Autopilot (PX4) initialization.
    ap = Autopilots.init!(;
        config = lockstep_config,
        libpath = libpath,
        home = home,
        edge_trigger = false,
    )
    try
        Autopilots.autopilot_step(
            ap,
            UInt64(0),
            vehicle.state.pos_ned,
            vehicle.state.vel_ned,
            vehicle.state.q_bn,
            vehicle.state.ω_body,
            Autopilots.AutopilotCommand();
            landed = true,
            battery = Powertrain.BatteryStatus(),
        )
        Autopilots.load_mission!(ap, mission_path)
        autopilot_src = Sources.LiveAutopilotSource(ap)

        # Timeline includes scenario AtTime events as boundaries.
        timeline = iris_timeline(
            t_end_s = t_end_s,
            dt_autopilot_s = dt_autopilot_s,
            dt_wind_s = dt_wind_s,
            dt_log_s = dt_log_s,
            dt_phys_s = dt_phys_s,
            scenario_source = scenario_src,
        )

        plant0 = Plant.init_plant_state(
            vehicle.state,
            vehicle.motor_actuators,
            vehicle.servo_actuators,
            vehicle.propulsion,
            battery,
        )

        if mode === :record
            rec_sink = Recording.InMemoryRecorder()
            eng = simulate(
                mode = :record,
                timeline = timeline,
                plant0 = plant0,
                dynfun = dynfun,
                integrator = integ,
                autopilot = autopilot_src,
                wind = wind_src,
                scenario = scenario_src,
                estimator = est_src,
                telemetry = telemetry,
                recorder = rec_sink,
                log_sinks = log_sinks,
            )


            rec = Recording.Tier0Recording(
                timeline = timeline,
                plant0 = plant0,
                recorder = rec_sink,
                meta = Dict{Symbol,Any}(:home => home, :mission => mission_path),
            )

            if recording_out !== nothing
                Recording.write_recording(recording_out, rec)
            end
            return rec
        end

        # Live (no recording)
        eng = simulate(
            mode = :live,
            timeline = timeline,
            plant0 = plant0,
            dynfun = dynfun,
            integrator = integ,
            autopilot = autopilot_src,
            wind = wind_src,
            scenario = scenario_src,
            estimator = est_src,
            telemetry = telemetry,
            recorder = Recording.NullRecorder(),
            log_sinks = log_sinks,
        )
        return eng
    finally
        Autopilots.close!(ap)
    end
end

export iris_default_home,
    iris_default_env_live,
    iris_default_env_replay,
    iris_default_vehicle,
    iris_default_battery,
    iris_default_contact,
    iris_default_scenario,
    iris_default_estimator,
    iris_dynfun,
    iris_reference_integrator,
    iris_integrator,
    iris_timeline,
    simulate_iris_mission
