using Test
using StaticArrays

const Sim = PX4Lockstep.Sim
const RT = Sim.Runtime
const REC = Sim.Recording

"""A minimal deterministic dynamics: x-acceleration from cmd.motors[1].

We use `RigidBodyState` only (attitude/body rates are held constant), so this exercises:
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
    @test traces.wind_ned.axis.t_us == timeline.wind.t_us
    @test traces.plant.axis.t_us == timeline.log.t_us

    @test traces.cmd.data == cmd_tr.data
    @test traces.wind_ned.data == wind_tr.data
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
    model = Sim.Vehicles.IrisQuadrotor()
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()
    propulsion = Sim.Propulsion.default_iris_quadrotor_set()
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


@testset "Recording.load_recording enforces schema version" begin
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
    REC.save_recording(path, rec)
    @test_throws ErrorException REC.load_recording(path)
end
