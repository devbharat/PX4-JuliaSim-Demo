using Test
using Random
using StaticArrays

const Sim = PX4Lockstep.Sim
const RT = Sim.Runtime
const REC = Sim.Recording

"""A minimal deterministic dynamics: x-acceleration from cmd.motors[1].

`RigidBodyState` is used exclusively (attitude/body rates are held constant), so this exercises:
- event-boundary traversal on `Timeline.evt`
- ZOH command sampling between autopilot ticks
- deterministic timeline traversal
"""
struct CmdAccelX end

function (f::CmdAccelX)(t::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
    a = u.cmd.motors[1]
    return Sim.RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Sim.Types.vec3(a, 0.0, 0.0),
        q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
end

"""Deterministic dynamics driven by wind samples (for record/replay tests)."""
struct WindAccelX end

function (f::WindAccelX)(t::Float64, x::Sim.RigidBody.RigidBodyState, u::Sim.Plant.PlantInput)
    a = u.wind_ned[1]
    return Sim.RigidBody.RigidBodyDeriv(
        pos_dot = x.vel_ned,
        vel_dot = Sim.Types.vec3(a, 0.0, 0.0),
        q_dot = Sim.RigidBody.quat_deriv(x.q_bn, x.ω_body),
        ω_dot = Sim.Types.vec3(0.0, 0.0, 0.0),
    )
end

"""A trivial open-loop autopilot source used to record commands."""
mutable struct ConstantMotorAutopilotSource
    cmd::Sim.Vehicles.ActuatorCommand
end

function RT.update!(src::ConstantMotorAutopilotSource, bus::RT.SimBus, plant, t_us::UInt64)
    bus.cmd = src.cmd
    return nothing
end

"""Scenario source that steps wind disturbance at a specified boundary."""
struct WindDistScenario
    t_step_us::UInt64
    dist_ned::Sim.Types.Vec3
end

function RT.update!(src::WindDistScenario, bus::RT.SimBus, plant, t_us::UInt64)
    bus.ap_cmd = Sim.Autopilots.AutopilotCommand()
    bus.landed = false
    bus.faults = Sim.Faults.FaultState()
    bus.wind_dist_ned =
        t_us >= src.t_step_us ? src.dist_ned : Sim.Types.vec3(0.0, 0.0, 0.0)
    return nothing
end



@testset "Runtime.Engine: replay integration matches analytic" begin
    # 0.1s run, autopilot is the densest axis so evt == ap (piecewise-constant inputs).
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
    )

    # Command: 0 for t < 0.05, 1 for t >= 0.05.
    cmd_data = Vector{Sim.Vehicles.ActuatorCommand}(undef, length(timeline.ap.t_us))
    for (i, t_us) in pairs(timeline.ap.t_us)
        a = (t_us < UInt64(50_000)) ? 0.0 : 1.0
        motors = SVector{12,Float64}(ntuple(j -> j == 1 ? a : 0.0, 12))
        cmd_data[i] = Sim.Vehicles.ActuatorCommand(motors = motors)
    end
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK4Integrator()
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)

    # Analytic: acceleration 0 for 0.05s, then 1 for 0.05s.
    @test isapprox(sim.plant.vel_ned[1], 0.05; atol = 1e-12)
    @test isapprox(sim.plant.pos_ned[1], 0.5 * (0.05^2); atol = 1e-12)
end


@testset "Runtime.Engine: record mode captures axis-aligned traces" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(50_000),
    )

    cmd_data = Vector{Sim.Vehicles.ActuatorCommand}(undef, length(timeline.ap.t_us))
    for (i, t_us) in pairs(timeline.ap.t_us)
        a = (t_us < UInt64(50_000)) ? 0.0 : 1.0
        motors = SVector{12,Float64}(ntuple(j -> j == 1 ? a : 0.0, 12))
        cmd_data[i] = Sim.Vehicles.ActuatorCommand(motors = motors)
    end
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    rec = REC.InMemoryRecorder()
    integ = Sim.Integrators.EulerIntegrator()

    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)

    sim = RT.plant_record_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
        recorder = rec,
    )
    RT.run!(sim)

    traces = REC.tier0_traces(rec, timeline)

    @test traces.cmd.axis.t_us == timeline.ap.t_us
    @test traces.wind_base_ned.axis.t_us == timeline.wind.t_us
    @test traces.plant.axis.t_us == timeline.log.t_us

    @test traces.cmd.data == cmd_tr.data
    @test traces.wind_base_ned.data == wind_tr.data
end


@testset "Runtime.Engine: record_estimator captures estimator stream" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(20_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    cmd_data = [Sim.Vehicles.ActuatorCommand() for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(1.0, 2.0, 3.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    rec = REC.InMemoryRecorder()
    Sim.simulate(
        mode = :record,
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Sim.Integrators.EulerIntegrator(),
        autopilot = Sim.Sources.ReplayAutopilotSource(cmd_tr),
        wind = Sim.Sources.ReplayWindSource(wind_tr),
        scenario = Sim.Sources.NullScenarioSource(),
        estimator = Sim.Sources.NullEstimatorSource(),
        recorder = rec,
        record_estimator = true,
    )

    traces = REC.estimator_traces(rec, timeline)
    @test traces.est.axis.t_us == timeline.ap.t_us
    @test traces.est isa REC.ZOHTrace
    @test traces.est.data[1].pos_ned == x0.pos_ned
    @test traces.est.data[end].pos_ned == x0.pos_ned
end


@testset "Runtime.Engine: strict_cmd validates even without sanitize" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_src = ConstantMotorAutopilotSource(cmd)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    scenario = Sim.Sources.NullScenarioSource()
    estimator = Sim.Sources.NullEstimatorSource()

    x0 = Sim.RigidBody.RigidBodyState()

    sim = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true, sanitize_cmd = false);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario,
        estimator = estimator,
    )

    @test_throws ErrorException RT.run!(sim)
end


@testset "Runtime.Engine: sanitize replaces NaN actuator commands" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(NaN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8,Float64}(ntuple(_ -> 0.0, 8))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_src = ConstantMotorAutopilotSource(cmd)
    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    scenario = Sim.Sources.NullScenarioSource()
    estimator = Sim.Sources.NullEstimatorSource()

    x0 = Sim.RigidBody.RigidBodyState()
    rec = REC.InMemoryRecorder()

    sim = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true, sanitize_cmd = true);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = Sim.Integrators.RK4Integrator(),
        autopilot = ap_src,
        wind = wind_src,
        scenario = scenario,
        estimator = estimator,
        recorder = rec,
    )

    RT.run!(sim)
    REC.finalize!(rec)

    traces = REC.tier0_traces(rec, timeline)
    @test traces.cmd.data[1].motors[1] == 0.0
end


@testset "Runtime.Engine: DirectActuators snap at autopilot ticks" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(ntuple(i -> i <= 4 ? 0.5 : 0.0, 12))
    cmd_data = [Sim.Vehicles.ActuatorCommand(motors = motors) for _ in timeline.ap.t_us]
    cmd_tr = REC.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, wind_data)

    env = Sim.Environment.EnvironmentModel(wind = Sim.Environment.NoWind())
    model = iris_vehicle_for_tests().model
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()
    propulsion = Sim.Propulsion.default_multirotor_set()
    battery = Sim.Powertrain.IdealBattery()

    rb0 = Sim.RigidBody.RigidBodyState()
    plant0 = Sim.Plant.init_plant_state(rb0, motor_act, servo_act, propulsion, battery)

    dynfun = Sim.PlantModels.CoupledMultirotorModel(
        model,
        env,
        Sim.Contacts.NoContact(),
        motor_act,
        servo_act,
        propulsion,
        battery,
    )

    integ = Sim.Integrators.RK4Integrator()
    ap_src = Sim.Sources.ReplayAutopilotSource(cmd_tr)
    wind_src = Sim.Sources.ReplayWindSource(wind_tr)
    sim = RT.plant_replay_engine(
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        autopilot = ap_src,
        wind = wind_src,
    )

    RT.run!(sim)

    @test sim.plant.rotor_ω[1] > 0.0
end



@testset "Record/replay equivalence at log ticks (Tier0)" begin
    # Record a short open-loop full-plant run, save/load the Tier0 recording, then replay the
    # recorded inputs and assert that the logged plant states match at every log tick.

    veh = iris_vehicle_for_tests()

    # Record with a non-zero "live" environment wind model, but replay with the
    # canonical replay environment (NoWind). If the plant ever reads env.wind
    # directly, this test will diverge.
    env_replay = iris_env_replay_for_tests()
    env_record = Sim.Environment.EnvironmentModel(
        atmosphere = env_replay.atmosphere,
        wind = Sim.Environment.ConstantWind(Sim.Types.vec3(5.0, 0.0, 0.0)),
        gravity = env_replay.gravity,
        origin = env_replay.origin,
    )
    battery = iris_battery_for_tests()
    contact = Sim.Contacts.NoContact()

    model_record = Sim.PlantModels.CoupledMultirotorModel(
        veh.model,
        env_record,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    model_replay = Sim.PlantModels.CoupledMultirotorModel(
        veh.model,
        env_replay,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    rb0 = Sim.RigidBody.RigidBodyState(pos_ned = Sim.Types.vec3(0.0, 0.0, -10.0))
    plant0 = Sim.Plant.init_plant_state(
        rb0,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        battery,
    )

    t_end_us = UInt64(50_000)
    tl = RT.build_timeline(
        UInt64(0),
        t_end_us;
        dt_ap_us = UInt64(2_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    # Constant motor duty command on the first 4 channels (rest unused).
    motors = SVector{12, Float64}(0.55, 0.55, 0.55, 0.55, 0, 0, 0, 0, 0, 0, 0, 0)
    servos = SVector{8, Float64}(0, 0, 0, 0, 0, 0, 0, 0)
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_live = ConstantMotorAutopilotSource(cmd)

    # Record wind from the live wind model.
    wind_live = Sim.Sources.LiveWindSource(env_record.wind, Random.MersenneTwister(0), 0.01)

    scenario = Sim.Sources.NullScenarioSource()
    estimator = Sim.Sources.NullEstimatorSource()

    integ = Sim.Integrators.RK4Integrator()

    # --- Record ---
    rec1 = REC.InMemoryRecorder()
    sim1 = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, enable_derived_outputs = true, record_estimator = false, strict_cmd = true);
        timeline = tl,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = plant0,
        dynfun = model_record,
        integrator = integ,
        autopilot = ap_live,
        wind = wind_live,
        scenario = scenario,
        estimator = estimator,
        recorder = rec1,
    )

    RT.run!(sim1)
    REC.finalize!(rec1)

    tier0 = REC.Tier0Recording(recorder = rec1, timeline = tl, plant0 = plant0)
    REC.validate_recording(tier0)

    # Persist and reload (exercises schema + I/O path).
    mktemp() do path, io
        close(io)
        REC.write_recording(path, tier0)
        tier0_loaded = REC.read_recording(path)
        REC.validate_recording(tier0_loaded)

        # Build replay sources from the loaded recording.
        tr = REC.tier0_traces(tier0_loaded.recorder, tier0_loaded.timeline)
        scn = REC.scenario_traces(tier0_loaded.recorder, tier0_loaded.timeline)

        ap_replay = Sim.Sources.ReplayAutopilotSource(tr.cmd)
        wind_replay = Sim.Sources.ReplayWindSource(tr.wind_base_ned)
        wind_dist = hasproperty(scn, :wind_dist) ? scn.wind_dist : nothing
        scenario_replay = Sim.Sources.ReplayScenarioSource(
            scn.ap_cmd,
            scn.landed,
            scn.faults;
            wind_dist = wind_dist,
        )

        # --- Replay (record again to compare traces) ---
        rec2 = REC.InMemoryRecorder()
        sim2 = RT.Engine(
            RT.EngineConfig(mode = RT.MODE_RECORD, enable_derived_outputs = true, record_estimator = false, strict_cmd = true);
            timeline = tl,
            bus = RT.SimBus(time_us = UInt64(0)),
            plant0 = plant0,
            dynfun = model_replay,
            integrator = integ,
            autopilot = ap_replay,
            wind = wind_replay,
            scenario = scenario_replay,
            estimator = estimator,
            recorder = rec2,
        )

        RT.run!(sim2)
        REC.finalize!(rec2)

        tr2 = REC.tier0_traces(rec2, tl)

        # Compare the logged plant state at every log tick.
        V = Sim.Verification
        @test length(tr.plant.data) == length(tr2.plant.data)
        for i in eachindex(tr.plant.data)
            e = V.plant_error(tr.plant.data[i], tr2.plant.data[i])
            @test e.pos <= 1e-12
            @test e.vel <= 1e-12
            @test e.att_rad <= 1e-12
            @test e.ω <= 1e-12
            @test e.rotor <= 1e-12
            @test e.soc <= 1e-15
            @test e.v1 <= 1e-12
        end
    end
end

@testset "Record/replay equivalence with wind disturbance (Tier0)" begin
    # Ensure non-zero wind_dist_ned (stepped at a non-wind boundary) reproduces exactly.
    t0_us = UInt64(0)
    t_end_us = UInt64(80_000)
    timeline = RT.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(5_000),
        dt_wind_us = UInt64(20_000),
        dt_log_us = UInt64(10_000),
    )

    base_wind = [Sim.Types.vec3(3.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = REC.SampleHoldTrace(timeline.wind, base_wind)

    motors = SVector{12, Float64}(ntuple(_ -> 0.0, 12))
    servos = SVector{8, Float64}(ntuple(_ -> 0.0, 8))
    cmd = Sim.Vehicles.ActuatorCommand(motors = motors, servos = servos)

    ap_live = ConstantMotorAutopilotSource(cmd)
    wind_live = Sim.Sources.ReplayWindSource(wind_tr)
    scenario = WindDistScenario(UInt64(35_000), Sim.Types.vec3(2.0, 0.0, 0.0))
    estimator = Sim.Sources.NullEstimatorSource()

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK4Integrator()

    # --- Record ---
    rec1 = REC.InMemoryRecorder()
    sim1 = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = WindAccelX(),
        integrator = integ,
        autopilot = ap_live,
        wind = wind_live,
        scenario = scenario,
        estimator = estimator,
        recorder = rec1,
    )

    RT.run!(sim1)
    REC.finalize!(rec1)

    tier0 = REC.Tier0Recording(recorder = rec1, timeline = timeline, plant0 = x0)
    REC.validate_recording(tier0)

    tr = REC.tier0_traces(rec1, timeline)
    scn = REC.scenario_traces(rec1, timeline)

    # --- Replay ---
    ap_replay = Sim.Sources.ReplayAutopilotSource(tr.cmd)
    wind_replay = Sim.Sources.ReplayWindSource(tr.wind_base_ned)
    scenario_replay = Sim.Sources.ReplayScenarioSource(
        scn.ap_cmd,
        scn.landed,
        scn.faults;
        wind_dist = scn.wind_dist,
    )

    rec2 = REC.InMemoryRecorder()
    sim2 = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_RECORD, strict_cmd = true);
        timeline = timeline,
        bus = RT.SimBus(time_us = UInt64(0)),
        plant0 = x0,
        dynfun = WindAccelX(),
        integrator = integ,
        autopilot = ap_replay,
        wind = wind_replay,
        scenario = scenario_replay,
        estimator = estimator,
        recorder = rec2,
    )

    RT.run!(sim2)
    REC.finalize!(rec2)

    tr2 = REC.tier0_traces(rec2, timeline)

    @test length(tr.plant.data) == length(tr2.plant.data)
    for i in eachindex(tr.plant.data)
        a = tr.plant.data[i]
        b = tr2.plant.data[i]
        @test isapprox(a.pos_ned, b.pos_ned; atol = 1e-12)
        @test isapprox(a.vel_ned, b.vel_ned; atol = 1e-12)
        @test isapprox(a.q_bn, b.q_bn; atol = 1e-12)
        @test isapprox(a.ω_body, b.ω_body; atol = 1e-12)
    end
end


@testset "Stage ordering: scenario -> wind -> derived outputs -> autopilot" begin
    # The canonical engine should guarantee that a scenario mutation at a boundary
    # is visible to the wind source at the same boundary, and that derived outputs
    # are computed before the autopilot runs.

    # Scenario: disconnect the battery and inject a step gust at t=0.
    mutable struct OrderScenario{W}
        wind::W
        dv_ned::Sim.Types.Vec3
        duration_s::Float64
        fired::Bool
    end

    function RT.update!(src::OrderScenario, bus::RT.SimBus, plant_state, t_us::UInt64)
        if !src.fired && t_us == 0
            src.fired = true
            bus.faults = Sim.Faults.set_battery_connected(bus.faults, false)
            Sim.Environment.add_step_gust!(src.wind, src.dv_ned, t_us, src.duration_s)
        end
        return nothing
    end

    mutable struct CaptureAutopilot
        seen_t_us::Vector{UInt64}
        seen_wind::Vector{Sim.Types.Vec3}
        seen_batt_connected::Vector{Bool}
    end

    function RT.update!(src::CaptureAutopilot, bus::RT.SimBus, plant, t_us::UInt64)
        push!(src.seen_t_us, t_us)
        push!(src.seen_wind, bus.wind_ned)
        push!(src.seen_batt_connected, bus.batteries[1].connected)
        bus.cmd = Sim.Vehicles.ActuatorCommand() # don't care
        return nothing
    end

    veh = iris_vehicle_for_tests()
    batt = iris_battery_for_tests()
    env0 = iris_env_replay_for_tests()
    wind_model = Sim.Environment.OUWind(
        mean = Sim.Types.vec3(0.0, 0.0, 0.0),
        σ = Sim.Types.vec3(0.0, 0.0, 0.0), # deterministic
        τ_s = 1.0,
    )
    env = Sim.Environment.EnvironmentModel(
        atmosphere = env0.atmosphere,
        wind = wind_model,
        gravity = env0.gravity,
        origin = env0.origin,
    )

    contact = Sim.Contacts.NoContact()
    model = Sim.PlantModels.CoupledMultirotorModel(
        veh.model,
        env,
        contact,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )
    rb0 = Sim.RigidBody.RigidBodyState()
    plant0 = Sim.Plant.init_plant_state(
        rb0,
        veh.motor_actuators,
        veh.servo_actuators,
        veh.propulsion,
        batt,
    )

    tl = RT.build_timeline(UInt64(0), UInt64(10_000);
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    scenario = OrderScenario(wind_model, Sim.Types.vec3(3.0, 0.0, 0.0), 1.0, false)
    wind_src = Sim.Sources.LiveWindSource(wind_model, Random.Xoshiro(1), 0.01)
    autopilot = CaptureAutopilot(UInt64[], Sim.Types.Vec3[], Bool[])

    eng = RT.Engine(
        RT.EngineConfig(mode = RT.MODE_LIVE, enable_derived_outputs = true);
        timeline = tl,
        plant0 = plant0,
        dynfun = model,
        integrator = Sim.Integrators.RK4Integrator(),
        scenario = scenario,
        wind = wind_src,
        autopilot = autopilot,
        estimator = Sim.Sources.NullEstimatorSource(),
        telemetry = RT.NullTelemetry(),
    )

    RT.run!(eng)

    # Autopilot should have run at t=0 and should observe the scenario+wind effects.
    @test autopilot.seen_t_us[1] == 0
    @test autopilot.seen_wind[1] == Sim.Types.vec3(3.0, 0.0, 0.0)
    @test autopilot.seen_batt_connected[1] == false
end

@testset "Recording.read_recording enforces schema version" begin
    timeline = RT.build_timeline(UInt64(0), UInt64(1_000);
        dt_ap_us = UInt64(1_000),
        dt_wind_us = UInt64(1_000),
        dt_log_us = UInt64(1_000),
    )
    rec = REC.Tier0Recording(
        bus_schema_version = RT.BUS_SCHEMA_VERSION - 1,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        recorder = REC.InMemoryRecorder(),
    )
    path = joinpath(mktempdir(), "bad_recording.jls")
    REC.write_recording(path, rec)
    @test_throws ErrorException REC.read_recording(path)
end
