"""Compare integrators recording workflow results (used for precompile and tests)."""
function compare_integrators_recording_results()
    t_end_us = UInt64(200_000)
    timeline = Runtime.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(10_000),
        dt_phys_us = UInt64(10_000),
    )

    half_us = t_end_us ÷ 2
    cmds = [
        Vehicles.ActuatorCommand(
            motors = SVector{12,Float64}(ntuple(_ -> (t >= half_us ? 1.0 : 0.0), 12)),
            servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8)),
        )
        for t in timeline.ap.t_us
    ]
    winds = [Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]

    cmd_trace = Recording.ZOHTrace(timeline.ap, cmds)
    wind_trace = Recording.SampleHoldTrace(timeline.wind, winds)

    ap_src = Sources.ReplayAutopilotSource(cmd_trace)
    wind_src = Sources.ReplayWindSource(wind_trace)

    x0 = RigidBody.RigidBodyState(
        pos_ned = Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Types.vec3(0.0, 0.0, 0.0),
        q_bn = Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Types.vec3(0.0, 0.0, 0.0),
    )

    rec = Recording.InMemoryRecorder()
    rec_sim = Runtime.plant_record_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = Sources.NullScenarioSource(),
        estimator = Sources.NullEstimatorSource(),
        telemetry = Runtime.NullTelemetry(),
        recorder = rec,
    )
    Runtime.run!(rec_sim)

    recording = Recording.Tier0Recording(timeline = timeline, plant0 = x0, recorder = rec)

    solvers = [
        "euler" => Integrators.EulerIntegrator(),
        "rk4" => Integrators.RK4Integrator(),
    ]

    workflows = Workflows
    out1 = mktempdir()
    out_csv1 = joinpath(out1, "summary.csv")
    rows1 = workflows.compare_integrators_recording(
        recording = recording,
        dynfun = CmdAccelX(),
        solvers = solvers,
        reference_integrator = Integrators.RK4Integrator(),
        out_csv = out_csv1,
        print_table = false,
    )

    out2 = mktempdir()
    out_csv2 = joinpath(out2, "summary.csv")
    rows2 = workflows.compare_integrators_recording(
        recording = recording,
        dynfun = CmdAccelX(),
        solvers = solvers,
        reference_integrator = Integrators.RK4Integrator(),
        out_csv = out_csv2,
        print_table = false,
    )

    return (rows1 = rows1, rows2 = rows2)
end

"""Record/replay equivalence at log ticks (Tier0)."""
function record_replay_equivalence_log_ticks()
    veh = iris_vehicle_for_tests()
    env_replay = iris_env_replay_for_tests()
    env_record = Environment.EnvironmentModel(
        atmosphere = env_replay.atmosphere,
        wind = Environment.ConstantWind(Types.vec3(5.0, 0.0, 0.0)),
        gravity = env_replay.gravity,
        origin = env_replay.origin,
    )
    battery = iris_battery_for_tests()
    contact = Contacts.NoContact()

    model_record = PlantModels.CoupledMultirotorModel(
        veh.model,
        env_record,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    model_replay = PlantModels.CoupledMultirotorModel(
        veh.model,
        env_replay,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    rb0 = RigidBody.RigidBodyState(pos_ned = Types.vec3(0.0, 0.0, -10.0))
    plant0 = Plant.init_plant_state(
        rb0,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    t_end_us = UInt64(50_000)
    tl = Runtime.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = UInt64(2_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(0.55, 0.55, 0.55, 0.55, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8,Float64}(0, 0, 0, 0, 0, 0, 0, 0)
    cmd = Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_live = ConstantMotorAutopilotSource(cmd)
    wind_live = Sources.LiveWindSource(env_record.wind, MersenneTwister(0), 0.01)

    scenario = Sources.NullScenarioSource()
    estimator = Sources.NullEstimatorSource()
    integ = Integrators.RK4Integrator()

    rec1 = Recording.InMemoryRecorder()
    sim1 = Runtime.Engine(
        Runtime.EngineConfig(mode = Runtime.MODE_RECORD, enable_derived_outputs = true, record_estimator = false, strict_cmd = true);
        timeline = tl,
        bus = Runtime.SimBus(time_us = UInt64(0)),
        plant0 = plant0,
        dynfun = model_record,
        integrator = integ,
        autopilot = ap_live,
        wind = wind_live,
        scenario = scenario,
        estimator = estimator,
        recorder = rec1,
    )

    Runtime.run!(sim1)
    Recording.finalize!(rec1)

    tier0 = Recording.Tier0Recording(recorder = rec1, timeline = tl, plant0 = plant0)
    Recording.validate_recording(tier0)

    return mktemp() do path, io
        close(io)
        Recording.write_recording(path, tier0)
        tier0_loaded = Recording.read_recording(path)
        Recording.validate_recording(tier0_loaded)

        tr = Recording.tier0_traces(tier0_loaded.recorder, tier0_loaded.timeline)
        scn = Recording.scenario_traces(tier0_loaded.recorder, tier0_loaded.timeline)

        ap_replay = Sources.ReplayAutopilotSource(tr.cmd)
        wind_replay = Sources.ReplayWindSource(tr.wind_base_ned)
        wind_dist = hasproperty(scn, :wind_dist) ? scn.wind_dist : nothing
        scenario_replay = Sources.ReplayScenarioSource(
            scn.ap_cmd,
            scn.landed,
            scn.faults;
            wind_dist = wind_dist,
        )

        rec2 = Recording.InMemoryRecorder()
        sim2 = Runtime.Engine(
            Runtime.EngineConfig(mode = Runtime.MODE_RECORD, enable_derived_outputs = true, record_estimator = false, strict_cmd = true);
            timeline = tl,
            bus = Runtime.SimBus(time_us = UInt64(0)),
            plant0 = plant0,
            dynfun = model_replay,
            integrator = integ,
            autopilot = ap_replay,
            wind = wind_replay,
            scenario = scenario_replay,
            estimator = estimator,
            recorder = rec2,
        )

        Runtime.run!(sim2)
        Recording.finalize!(rec2)

        tr2 = Recording.tier0_traces(rec2, tl)

        max_pos = 0.0
        max_vel = 0.0
        max_att = 0.0
        max_ω = 0.0
        max_rotor = 0.0
        max_soc = 0.0
        max_v1 = 0.0

        for i in eachindex(tr.plant.data)
            e = Verification.plant_error(tr.plant.data[i], tr2.plant.data[i])
            max_pos = max(max_pos, e.pos)
            max_vel = max(max_vel, e.vel)
            max_att = max(max_att, e.att_rad)
            max_ω = max(max_ω, e.ω)
            max_rotor = max(max_rotor, e.rotor)
            max_soc = max(max_soc, e.soc)
            max_v1 = max(max_v1, e.v1)
        end

        return (
            max_pos = max_pos,
            max_vel = max_vel,
            max_att = max_att,
            max_ω = max_ω,
            max_rotor = max_rotor,
            max_soc = max_soc,
            max_v1 = max_v1,
            count = length(tr.plant.data),
        )
    end
end

"""Record/replay equivalence with wind disturbance (Tier0)."""
function record_replay_equivalence_wind_disturbance()
    t0_us = UInt64(0)
    t_end_us = UInt64(80_000)
    timeline = Runtime.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(5_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(10_000),
    )

    base_wind = [Types.vec3(3.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = Recording.SampleHoldTrace(timeline.wind, base_wind)

    motors = SVector{12,Float64}(ntuple(_ -> 0.0, 12))
    servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8))
    cmd = Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_live = ConstantMotorAutopilotSource(cmd)
    wind_live = Sources.ReplayWindSource(wind_tr)
    scenario = WindDistScenario(UInt64(35_000), Types.vec3(2.0, 0.0, 0.0))
    estimator = Sources.NullEstimatorSource()

    x0 = RigidBody.RigidBodyState(
        pos_ned = Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Types.vec3(0.0, 0.0, 0.0),
        q_bn = Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Integrators.RK4Integrator()

    rec1 = Recording.InMemoryRecorder()
    sim1 = Runtime.Engine(
        Runtime.EngineConfig(mode = Runtime.MODE_RECORD, strict_cmd = true);
        timeline = timeline,
        bus = Runtime.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = WindAccelX(),
        integrator = integ,
        autopilot = ap_live,
        wind = wind_live,
        scenario = scenario,
        estimator = estimator,
        recorder = rec1,
    )

    Runtime.run!(sim1)
    Recording.finalize!(rec1)

    tier0 = Recording.Tier0Recording(recorder = rec1, timeline = timeline, plant0 = x0)
    Recording.validate_recording(tier0)

    tr = Recording.tier0_traces(rec1, timeline)
    scn = Recording.scenario_traces(rec1, timeline)

    ap_replay = Sources.ReplayAutopilotSource(tr.cmd)
    wind_replay = Sources.ReplayWindSource(tr.wind_base_ned)
    scenario_replay = Sources.ReplayScenarioSource(
        scn.ap_cmd,
        scn.landed,
        scn.faults;
        wind_dist = scn.wind_dist,
    )

    rec2 = Recording.InMemoryRecorder()
    sim2 = Runtime.Engine(
        Runtime.EngineConfig(mode = Runtime.MODE_RECORD, strict_cmd = true);
        timeline = timeline,
        bus = Runtime.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = WindAccelX(),
        integrator = integ,
        autopilot = ap_replay,
        wind = wind_replay,
        scenario = scenario_replay,
        estimator = estimator,
        recorder = rec2,
    )

    Runtime.run!(sim2)
    Recording.finalize!(rec2)

    tr2 = Recording.tier0_traces(rec2, timeline)

    max_pos = 0.0
    max_vel = 0.0
    max_q = 0.0
    max_ω = 0.0

    for i in eachindex(tr.plant.data)
        a = tr.plant.data[i]
        b = tr2.plant.data[i]
        max_pos = max(max_pos, maximum(abs.(a.pos_ned .- b.pos_ned)))
        max_vel = max(max_vel, maximum(abs.(a.vel_ned .- b.vel_ned)))
        max_q = max(max_q, maximum(abs.(a.q_bn .- b.q_bn)))
        max_ω = max(max_ω, maximum(abs.(a.ω_body .- b.ω_body)))
    end

    return (max_pos = max_pos, max_vel = max_vel, max_q = max_q, max_ω = max_ω, count = length(tr.plant.data))
end

"""Build a small Tier0 recording and verify replay parity against spec-built replay."""
function iris_replay_parity_result()
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
    scenario_src = Sources.LiveScenarioSource(scenario_obj)

    timeline = iris_timeline_for_tests(
        t_end_s = t_end_s,
        dt_autopilot_s = dt_ap,
        dt_wind_s = dt_wind,
        dt_log_s = dt_log,
        dt_phys_s = nothing,
        scenario_source = scenario_src,
    )

    zero_cmd = Vehicles.ActuatorCommand()
    cmds = [zero_cmd for _ in timeline.ap.t_us]
    cmd_trace = Recording.ZOHTrace(timeline.ap, cmds)
    autopilot = Sources.ReplayAutopilotSource(cmd_trace)

    zero_wind = Types.vec3(0.0, 0.0, 0.0)
    winds = [zero_wind for _ in timeline.wind.t_us]
    wind_trace = Recording.SampleHoldTrace(timeline.wind, winds)
    wind = Sources.ReplayWindSource(wind_trace)

    plant0 = Plant.init_plant_state(
        vehicle.state,
        vehicle.motor_actuators,
        vehicle.servo_actuators,
        vehicle.propulsion,
        battery,
    )

    integ = integrator_from_symbol(:RK4)

    rec_sink = Recording.InMemoryRecorder()
    _ = Sim.simulate(
        mode = :record,
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        autopilot = autopilot,
        wind = wind,
        scenario = scenario_src,
        estimator = Sources.NullEstimatorSource(),
        telemetry = Runtime.NullTelemetry(),
        recorder = rec_sink,
    )

    rec = Recording.Tier0Recording(
        timeline = timeline,
        plant0 = plant0,
        recorder = rec_sink,
        meta = Dict{Symbol,Any}(),
    )

    base_spec = iris_spec_for_tests()
    spec = Aircraft.AircraftSpec(
        name = base_spec.name,
        px4 = base_spec.px4,
        timeline = Aircraft.TimelineSpec(
            t_end_s = t_end_s,
            dt_autopilot_s = dt_ap,
            dt_wind_s = dt_wind,
            dt_log_s = dt_log,
            dt_phys_s = nothing,
        ),
        plant = Aircraft.PlantSpec(integrator = :RK4, contact = contact),
        airframe = base_spec.airframe,
        actuation = base_spec.actuation,
        power = base_spec.power,
        sensors = base_spec.sensors,
        seed = base_spec.seed,
        home = home,
        telemetry = base_spec.telemetry,
        log_sinks = base_spec.log_sinks,
    )

    eng_spec = Aircraft.build_engine(spec; mode = :replay, recording_in = rec)

    traces = Recording.tier0_traces(rec)
    scn_tr = Recording.scenario_traces(rec)
    wind_dist = hasproperty(scn_tr, :wind_dist) ? scn_tr.wind_dist : nothing

    scenario_replay = Sources.ReplayScenarioSource(
        scn_tr.ap_cmd,
        scn_tr.landed,
        scn_tr.faults;
        wind_dist = wind_dist,
    )
    wind_replay = Sources.ReplayWindSource(traces.wind_base_ned)
    autopilot_replay = Sources.ReplayAutopilotSource(traces.cmd)

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
        estimator = Sources.NullEstimatorSource(),
        telemetry = Runtime.NullTelemetry(),
        recorder = Recording.NullRecorder(),
    )

    return eng_spec.plant == eng_manual.plant
end
