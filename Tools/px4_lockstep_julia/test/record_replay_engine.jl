using Test
using StaticArrays

const Sim = PX4Lockstep.Sim
const RR = Sim.RecordReplay

"""A minimal deterministic dynamics: x-acceleration from cmd.motors[1].

We use `RigidBodyState` only (attitude/body rates are held constant), so this exercises:
- record/replay scheduling
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


@testset "RecordReplay.BusEngine: replay integration matches analytic" begin
    # 0.1s run, autopilot is the densest axis so evt == ap (piecewise-constant inputs).
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RR.build_timeline(
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
    cmd_tr = RR.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = RR.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    integ = Sim.Integrators.RK4Integrator()
    sim = RR.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = integ,
        cmd_trace = cmd_tr,
        wind_trace = wind_tr,
    )

    RR.run!(sim)

    # Analytic: acceleration 0 for 0.05s, then 1 for 0.05s.
    @test isapprox(sim.plant.vel_ned[1], 0.05; atol = 1e-12)
    @test isapprox(sim.plant.pos_ned[1], 0.5 * (0.05^2); atol = 1e-12)
end


@testset "RecordReplay.BusEngine: record mode captures axis-aligned traces" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(100_000)
    timeline = RR.build_timeline(
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
    cmd_tr = RR.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = RR.SampleHoldTrace(timeline.wind, wind_data)

    x0 = Sim.RigidBody.RigidBodyState(
        pos_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        vel_ned = Sim.Types.vec3(0.0, 0.0, 0.0),
        q_bn = Sim.Types.Quat(1.0, 0.0, 0.0, 0.0),
        ω_body = Sim.Types.vec3(0.0, 0.0, 0.0),
    )

    rec = RR.InMemoryRecorder()
    integ = Sim.Integrators.EulerIntegrator()
    sim = RR.plant_replay_engine(
        timeline = timeline,
        plant0 = x0,
        dynfun = CmdAccelX(),
        integrator = integ,
        cmd_trace = cmd_tr,
        wind_trace = wind_tr,
        recorder = rec,
    )

    sim.cfg = RR.EngineConfig(mode = RR.MODE_RECORD)
    RR.run!(sim)

    traces = RR.tier0_traces(rec, timeline)

    @test traces.cmd.axis.t_us == timeline.ap.t_us
    @test traces.wind_ned.axis.t_us == timeline.wind.t_us
    @test traces.plant.axis.t_us == timeline.log.t_us

    @test traces.cmd.data == cmd_tr.data
    @test traces.wind_ned.data == wind_tr.data
end


@testset "RecordReplay.BusEngine: DirectActuators snap at autopilot ticks" begin
    t0_us = UInt64(0)
    t_end_us = UInt64(10_000)
    timeline = RR.build_timeline(
        t0_us,
        t_end_us;
        dt_ap_us = UInt64(10_000),
        dt_wind_us = UInt64(10_000),
        dt_log_us = UInt64(10_000),
    )

    motors = SVector{12,Float64}(ntuple(i -> i <= 4 ? 0.5 : 0.0, 12))
    cmd_data = [Sim.Vehicles.ActuatorCommand(motors = motors) for _ in timeline.ap.t_us]
    cmd_tr = RR.ZOHTrace(timeline.ap, cmd_data)

    wind_data = [Sim.Types.vec3(0.0, 0.0, 0.0) for _ in timeline.wind.t_us]
    wind_tr = RR.SampleHoldTrace(timeline.wind, wind_data)

    env = Sim.Environment.EnvironmentModel(wind = Sim.Environment.NoWind())
    model = Sim.Vehicles.IrisQuadrotor()
    motor_act = Sim.Vehicles.DirectActuators()
    servo_act = Sim.Vehicles.DirectActuators()
    propulsion = Sim.Propulsion.default_iris_quadrotor_set()
    battery = Sim.Powertrain.IdealBattery()

    rb0 = Sim.RigidBody.RigidBodyState()
    plant0 = Sim.Plant.init_plant_state(rb0, motor_act, servo_act, propulsion, battery)

    dynfun = Sim.PlantSimulation.PlantDynamicsWithContact(
        model,
        env,
        Sim.Contacts.NoContact(),
        motor_act,
        servo_act,
        propulsion,
        battery,
    )

    integ = Sim.Integrators.RK4Integrator()
    sim = RR.plant_replay_engine(
        timeline = timeline,
        plant0 = plant0,
        dynfun = dynfun,
        integrator = integ,
        cmd_trace = cmd_tr,
        wind_trace = wind_tr,
    )

    RR.run!(sim)

    @test sim.plant.rotor_ω[1] > 0.0
end


@testset "RecordReplay.load_recording enforces schema version" begin
    timeline = RR.build_timeline(UInt64(0), UInt64(1_000);
        dt_ap_us = UInt64(1_000),
        dt_wind_us = UInt64(1_000),
        dt_log_us = UInt64(1_000),
    )
    rec = RR.Tier0Recording(
        bus_schema_version = RR.BUS_SCHEMA_VERSION - 1,
        timeline = timeline,
        plant0 = Sim.RigidBody.RigidBodyState(),
        recorder = RR.InMemoryRecorder(),
    )
    path = joinpath(mktempdir(), "bad_recording.jls")
    RR.save_recording(path, rec)
    @test_throws ErrorException RR.load_recording(path)
end
