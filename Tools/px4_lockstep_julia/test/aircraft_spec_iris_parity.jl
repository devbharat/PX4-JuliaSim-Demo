using Test

@testset "AircraftSpec: Iris replay parity (Phase 1)" begin
    # Build a tiny deterministic Tier-0 recording without PX4 by running the engine
    # in :record mode with replay sources.

    t_end_s = 0.2
    dt_ap = 0.02
    dt_wind = 0.02
    dt_log = 0.02

    home = iris_home_for_tests()
    contact = iris_contact_for_tests()

    env = iris_env_replay_for_tests(home = home)
    vehicle = iris_vehicle_for_tests()
    battery = iris_battery_for_tests()
    dynfun = iris_dynfun_for_tests(env, vehicle, battery; contact = contact)

    scenario_obj = iris_scenario_for_tests()
    scenario_src = Sim.Sources.LiveScenarioSource(scenario_obj)

    timeline = iris_timeline_for_tests(
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

    integ = integrator_from_symbol(:RK4)

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
    base_spec = iris_spec_for_tests()
    spec = Sim.Aircraft.AircraftSpec(
        name = base_spec.name,
        px4 = base_spec.px4,
        timeline = Sim.Aircraft.TimelineSpec(
            t_end_s = t_end_s,
            dt_autopilot_s = dt_ap,
            dt_wind_s = dt_wind,
            dt_log_s = dt_log,
            dt_phys_s = nothing,
        ),
        plant = Sim.Aircraft.PlantSpec(integrator = :RK4, contact = contact),
        airframe = base_spec.airframe,
        actuation = base_spec.actuation,
        power = base_spec.power,
        sensors = base_spec.sensors,
        seed = base_spec.seed,
        home = home,
        telemetry = base_spec.telemetry,
        log_sinks = base_spec.log_sinks,
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

    env_replay = iris_env_replay_for_tests(home = home)
    vehicle_replay = iris_vehicle_for_tests(; x0 = rec.plant0.rb)
    battery_replay = iris_battery_for_tests()
    dynfun_replay = iris_dynfun_for_tests(env_replay, vehicle_replay, battery_replay; contact = contact)

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
