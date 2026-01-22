using Test

@testset "AircraftSpec: Iris replay parity (Phase 1)" begin
    # Build a tiny deterministic Tier-0 recording without PX4 by running the engine
    # in :record mode with replay sources.

    t_end_s = 0.2
    dt_ap = 0.02
    dt_wind = 0.02
    dt_log = 0.02

    home = Sim.iris_default_home()
    contact = Sim.iris_default_contact()

    env = Sim.iris_default_env_replay(home = home)
    vehicle = Sim.iris_default_vehicle()
    battery = Sim.iris_default_battery()
    dynfun = Sim.iris_dynfun(env, vehicle, battery; contact = contact)

    scenario_obj = Sim.iris_default_scenario()
    scenario_src = Sim.Sources.LiveScenarioSource(scenario_obj)

    timeline = Sim.iris_timeline(
        t_end_s = t_end_s,
        dt_autopilot_s = dt_ap,
        dt_wind_s = dt_wind,
        dt_log_s = dt_log,
        dt_phys_s = nothing,
        scenario_source = scenario_src,
    )

    # Constant command trace (zeros) aligned to timeline.ap
    zero_cmd = Sim.Vehicles.ActuatorCommand()
    cmds = [zero_cmd for _ in timeline.ap.t_us]
    cmd_trace = Sim.Recording.ZOHTrace(timeline.ap, cmds)
    autopilot = Sim.Sources.ReplayAutopilotSource(cmd_trace)

    # Constant wind trace (zero) aligned to timeline.wind
    zero_wind = Sim.Types.vec3(0.0, 0.0, 0.0)
    winds = [zero_wind for _ in timeline.wind.t_us]
    wind_trace = Sim.Recording.SampleHoldTrace(timeline.wind, winds)
    wind = Sim.Sources.ReplayWindSource(wind_trace)

    plant0 = Sim.Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    integ = Sim.iris_integrator(:RK4)

    rec_sink = Sim.Recording.InMemoryRecorder()
    _ = Sim.simulate(
        mode = :record,
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        autopilot = autopilot,
        wind = wind,
        scenario = scenario_src,
        estimator = Sim.Sources.NullEstimatorSource(),
        telemetry = Sim.Runtime.NullTelemetry(),
        recorder = rec_sink,
    )

    rec = Sim.Recording.Tier0Recording(
        timeline = timeline,
        plant0 = plant0,
        recorder = rec_sink,
        meta = Dict{Symbol,Any}(),
    )

    # --- Replay via the new spec builder ---
    spec = Sim.Aircraft.iris_spec(
        t_end_s = t_end_s,
        dt_autopilot_s = dt_ap,
        dt_wind_s = dt_wind,
        dt_log_s = dt_log,
        integrator = :RK4,
        home = home,
        contact = contact,
    )

    eng_spec = Sim.Aircraft.build_engine(spec; mode = :replay, recording_in = rec)

    # --- Manual replay (baseline) ---
    traces = Sim.Recording.tier0_traces(rec)
    scn_tr = Sim.Recording.scenario_traces(rec)
    wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing

    scenario_replay = Sim.Sources.ReplayScenarioSource(
        scn_tr.ap_cmd,
        scn_tr.landed,
        scn_tr.faults;
        wind_dist = wind_dist,
    )
    wind_replay = Sim.Sources.ReplayWindSource(traces.wind_ned)
    autopilot_replay = Sim.Sources.ReplayAutopilotSource(traces.cmd)

    env_replay = Sim.iris_default_env_replay(home = home)
    vehicle_replay = Sim.iris_default_vehicle(; x0 = rec.plant0.rb)
    battery_replay = Sim.iris_default_battery()
    dynfun_replay = Sim.iris_dynfun(env_replay, vehicle_replay, battery_replay; contact = contact)

    eng_manual = Sim.simulate(
        mode = :replay,
        timeline = rec.timeline,
        plant0 = rec.plant0,
        dynfun = dynfun_replay,
        integrator = integ,
        autopilot = autopilot_replay,
        wind = wind_replay,
        scenario = scenario_replay,
        estimator = Sim.Sources.NullEstimatorSource(),
        telemetry = Sim.Runtime.NullTelemetry(),
        recorder = Sim.Recording.NullRecorder(),
    )

    @test eng_spec.plant == eng_manual.plant
end
